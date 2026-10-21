"""
AXIS-OS — Hôte d'app : le programme du processus d'une app
===========================================================
Chaque appel à une app (on_start, run_mission, on_stop) tourne
dans un interpréteur Python NEUF, lancé par processus_app.py avec

    python -I hote_app.py <descripteur du pont> <racine d'AXIS-OS> <pid de l'OS>

Pourquoi un interpréteur neuf plutôt qu'une copie du processus de l'OS
(fork) : une copie hérite de toute sa mémoire (objets HAL, clés) et de
ses connexions ouvertes (sur un vrai G1, les sockets DDS vers les
moteurs). Ici, le processus ne reçoit qu'une extrémité du pont IPC (une
socket Unix) et ses trois flux standard ; « -I » lui fait ignorer les
variables PYTHON* et le dossier utilisateur, et processus_app.py ne lui
transmet qu'un environnement minimal, sans aucun secret.

Déroulé :
  1. lire le message de départ (JSON, voir SCHEMA_DEPART) : quel hook,
     quelle app, quels paramètres ;
  2. recharger le code de l'app depuis sa source (voir _charger_classe) ;
  3. exécuter le hook avec un HALBridgeClient, qui envoie chaque appel au
     robot par le pont ;
  4. renvoyer le résultat en JSON (message « fin »), puis se terminer.

Tout ce que ce processus renvoie est revérifié par l'OS : il est sous le
contrôle de l'app, donc non fiable.

Licence : Apache 2.0, comme tout le dépôt (voir LICENSE).
"""

import sys


def _litteral(noeud) -> bool:
    import ast
    try:
        ast.literal_eval(noeud)
        return True
    except (ValueError, TypeError, SyntaxError, MemoryError, RecursionError):
        return False


def _cibles_simples(noeud) -> bool:
    """Vrai si l'affectation ne vise que des noms (CONSTANTE = 3), jamais
    un attribut ou un élément (objet.champ = 3), qui modifierait un objet
    créé par du code non repris."""
    import ast
    cibles = noeud.targets if isinstance(noeud, ast.Assign) else [noeud.target]
    pile = list(cibles)
    while pile:
        c = pile.pop()
        if isinstance(c, (ast.Tuple, ast.List)):
            pile.extend(c.elts)
        elif not isinstance(c, ast.Name):
            return False
    return True


def _definitions_seulement(source: str, nom_fichier: str):
    """Compile un script en ne gardant que ses imports, ses définitions de
    fonctions et de classes, ses docstrings et ses constantes littérales.

    Pourquoi : une app définie dans un script lancé directement (un script
    de test, « python3 test/test_x.py ») vit dans le module __main__ du
    processus de l'OS. Le processus neuf ne peut pas la retrouver par son
    nom de module, et réexécuter le script entier y relancerait tout le
    script : ses tests, ses runtimes, ses missions. Le code placé au niveau
    du module hors définitions (appels, boucles, if __name__ == ..., et
    affectations autres que NOM = littéral) n'est donc pas repris.
    """
    import ast
    arbre = ast.parse(source, nom_fichier)
    gardes = []
    for noeud in arbre.body:
        if isinstance(noeud, (ast.Import, ast.ImportFrom, ast.FunctionDef,
                              ast.AsyncFunctionDef, ast.ClassDef)):
            gardes.append(noeud)
        elif isinstance(noeud, ast.Expr) and isinstance(noeud.value, ast.Constant):
            gardes.append(noeud)
        elif isinstance(noeud, (ast.Assign, ast.AnnAssign)) and noeud.value is not None \
                and _litteral(noeud.value) and _cibles_simples(noeud):
            gardes.append(noeud)
    arbre.body = gardes
    return compile(arbre, nom_fichier, "exec")


def _charger_module(spec: dict):
    """Rend le module décrit par `spec` (voir processus_app.SCHEMA_APP).

    « module » : import normal d'un module installé (apps livrées avec
    AXIS-OS). « source » : le texte exact vérifié par l'OS (signature) est
    exécuté dans un module neuf ; c'est le seul mode utilisé pour une app
    signée, pour que le code exécuté soit bien celui dont la signature a été
    vérifiée, et non le fichier tel qu'il est sur le disque au moment de la
    mission. « script » : voir _definitions_seulement().
    """
    import importlib
    import types
    genre = spec["genre"]
    if genre == "module":
        return importlib.import_module(spec["module"])
    nom = spec["module"]
    module = types.ModuleType(nom)
    module.__file__ = spec.get("fichier") or f"<{nom}>"
    if spec.get("paquet"):
        module.__package__ = spec["paquet"]
    sys.modules[nom] = module
    if genre == "source":
        code = compile(spec["source"], module.__file__, "exec")
    else:
        code = _definitions_seulement(spec["source"], module.__file__)
    exec(code, module.__dict__)
    return module


def _resoudre(module, chemin: str):
    obj = module
    for partie in chemin.split("."):
        obj = getattr(obj, partie, None)
    return obj


def _charger_classe(spec: dict):
    """Rend la classe RobotApp décrite par `spec`."""
    from axis_os.store.sdk.app_base import RobotApp
    module = _charger_module(spec)
    if spec.get("classe"):
        classe = _resoudre(module, spec["classe"])
    else:
        candidates = [v for v in vars(module).values()
                      if isinstance(v, type) and issubclass(v, RobotApp)
                      and v is not RobotApp and v.__module__ == module.__name__]
        classe = candidates[0] if len(candidates) == 1 else None
    if not (isinstance(classe, type) and issubclass(classe, RobotApp)):
        raise ImportError(f"classe d'app introuvable : {spec.get('classe') or '(unique RobotApp)'}")
    return classe


def _decoder_donnee(v):
    """Inverse de processus_app.encoder_donnee : reconstruit les
    énumérations, laisse le reste tel quel. Le message vient de l'OS."""
    import enum
    import importlib
    if isinstance(v, list):
        return [_decoder_donnee(x) for x in v]
    if isinstance(v, dict):
        if set(v) == {"$enum", "$nom"}:
            module_nom, _, qualname = v["$enum"].partition(":")
            classe = _resoudre(importlib.import_module(module_nom), qualname)
            if not (isinstance(classe, type) and issubclass(classe, enum.Enum)):
                raise TypeError(f"{v['$enum']} n'est pas une énumération")
            return classe[v["$nom"]]
        return {k: _decoder_donnee(x) for k, x in v.items()}
    return v


def _instancier(classe, etat):
    """Crée l'instance de l'app. `etat` est l'état de l'instance que l'OS
    avait reçue (voir processus_app.etat_json) : il est réappliqué après
    le constructeur, comme le faisait autrefois pickle, mais seulement
    pour les attributs qui sont des données."""
    try:
        app = classe()
    except TypeError:
        if etat is None:
            raise
        # Constructeur à arguments : même chemin que pickle, sans __init__.
        app = classe.__new__(classe)
    if etat:
        vars(app).update(_decoder_donnee(etat))
    return app


def _vers_json(valeur, profondeur=0):
    """Convertit ce qu'une app renvoie en données JSON pures : tuples et
    ensembles en listes, enums par leur nom, scalaires numpy en nombres.
    Tout autre objet est refusé (TypeError) plutôt que deviné."""
    import enum
    import math
    if profondeur > 30:
        raise TypeError("structure trop profonde")
    if valeur is None or isinstance(valeur, (bool, str)):
        return valeur
    if isinstance(valeur, int):
        return int(valeur)
    if isinstance(valeur, float):
        if not math.isfinite(valeur):
            raise TypeError("nombre non fini")
        return float(valeur)
    if isinstance(valeur, enum.Enum):
        return valeur.name
    if isinstance(valeur, dict):
        return {str(k): _vers_json(v, profondeur + 1) for k, v in valeur.items()}
    if isinstance(valeur, (list, tuple, set, frozenset)):
        return [_vers_json(v, profondeur + 1) for v in valeur]
    item = getattr(valeur, "item", None)  # scalaire numpy
    if callable(item) and getattr(valeur, "ndim", None) == 0:
        return _vers_json(item(), profondeur + 1)
    raise TypeError(f"{type(valeur).__name__} n'est pas une donnée JSON")


def _resultat_mission(resultat, journal):
    if resultat is None:
        return None
    return {
        "success": bool(resultat.success),
        "items_processed": int(resultat.items_processed or 0),
        "error_message": None if resultat.error_message is None else str(resultat.error_message),
        "metrics": _vers_json(resultat.metrics or {}),
        "log": [str(x) for x in journal],
    }


def _executer(app, depart, client):
    """Exécute le hook demandé et rend son résultat en JSON."""
    from axis_os.store.sdk.app_base import PermissionError, EstopActiveError, MissionResult

    hook = depart["hook"]
    if hook == "on_start":
        try:
            pret = app.on_start(client)
            return {"pret": pret is True or bool(pret), "log": client.get_mission_log()}
        except (PermissionError, EstopActiveError) as exc:
            return {"pret": False, "log": client.get_mission_log() + [f"on_start refusé : {exc}"]}

    if hook == "on_stop":
        try:
            app.on_stop(client)
            return {"ok": True, "log": client.get_mission_log()}
        except (PermissionError, EstopActiveError) as exc:
            return {"ok": False, "log": client.get_mission_log() + [f"on_stop refusé : {exc}"]}

    # run_mission
    try:
        resultat = app.run_mission(client, depart["params"] or {})
    except PermissionError as exc:
        resultat = MissionResult(success=False, mission_id="", app_id="", duration_s=0.0,
                                 error_message=f"Permission refusée : {exc}")
    except EstopActiveError as exc:
        resultat = MissionResult(success=False, mission_id="", app_id="", duration_s=0.0,
                                 error_message=f"Mission interrompue par e-stop : {exc}")
    return {"mission": _resultat_mission(resultat, client.get_mission_log())}


def _mourir_avec_l_os(pid_os: int) -> None:
    """Demande au noyau de tuer ce processus si celui de l'OS meurt
    (prctl PR_SET_PDEATHSIG, Linux). Sans cela, une app qui ne parle plus
    au pont continuerait de tourner après un arrêt brutal de l'OS."""
    import ctypes
    import os
    import signal
    try:
        libc = ctypes.CDLL(None, use_errno=True)
        libc.prctl(1, int(signal.SIGKILL), 0, 0, 0)     # 1 = PR_SET_PDEATHSIG
    except (OSError, AttributeError):
        pass
    if os.getppid() != pid_os:      # l'OS est mort avant le prctl
        os._exit(1)


def main(argv) -> int:
    fd, racine = int(argv[1]), argv[2]
    _mourir_avec_l_os(int(argv[3]))
    sys.path.insert(0, racine)
    from axis_os.store.runtime.canal_ipc import Canal
    from axis_os.store.runtime.processus_app import SCHEMA_DEPART

    # Reçu héritable (pass_fds) ; ne doit pas l'être pour un programme que
    # l'app lancerait (os.system, exec) : sorti du groupe de processus, il
    # garderait le pont ouvert après la mort de l'app.
    import os
    os.set_inheritable(fd, False)
    canal = Canal.depuis_descripteur(fd)
    depart = SCHEMA_DEPART.verifier(canal.recevoir(timeout=30.0))
    sys.path[:] = [racine] + [p for p in depart["sys_path"] if p != racine]

    try:
        if depart["hook"] == "fonction":
            # Fonction de confiance de l'OS (SandboxRunner.run) : aucun
            # accès au robot, seulement des quotas.
            fonction = _resoudre(_charger_module(depart["app"]), depart["app"]["fonction"])
            if not callable(fonction):
                raise ImportError(f"fonction introuvable : {depart['app']['fonction']}")
            valeur = fonction(*(depart["args"] or []), **(depart["kwargs"] or {}))
            resultat = {"valeur": _vers_json(valeur)}
        else:
            from axis_os.store.runtime.hal_bridge import HALBridgeClient
            client = HALBridgeClient(canal=canal, tolerance_lenteur=depart["tolerance_lenteur"])
            classe = _charger_classe(depart["app"])
            app = _instancier(classe, depart["etat"])
            resultat = _executer(app, depart, client)
        message = {"type": "fin", "resultat": resultat, "erreur": None}
    except Exception as exc:                # noqa: BLE001 — rapporté à l'OS
        message = {"type": "fin", "resultat": None,
                   "erreur": f"{type(exc).__name__}: {exc}"[:2000]}
    try:
        canal.envoyer(message)
    except (TypeError, ValueError) as exc:
        canal.envoyer({"type": "fin", "resultat": None,
                       "erreur": f"résultat non transmissible en JSON : {exc}"[:2000]})
    canal.fermer()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
