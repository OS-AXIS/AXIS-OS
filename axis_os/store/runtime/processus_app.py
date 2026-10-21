"""
AXIS-OS — Lancement du processus d'une app (côté OS)
=====================================================
Crée le processus neuf d'une app (voir hote_app.py, qui tourne dedans),
lui transmet de quoi recharger l'app, et relaie ce qu'elle affiche.

Ce que le processus de l'app reçoit, et rien d'autre :
  - une extrémité d'une socket Unix, le pont IPC (canal_ipc.py) ;
  - stdin sur /dev/null, stdout et stderr sur un tube lu par l'OS, qui
    réaffiche les lignes préfixées par l'app_id ;
  - un environnement réduit à PATH et à la langue : aucune variable de
    l'OS (jetons, clés, configuration DDS) ne lui parvient ;
  - sa propre session (start_new_session) : pas de terminal de contrôle.

subprocess.Popen(close_fds=True, pass_fds=...) ferme tous les autres
descripteurs du processus de l'OS dans le nouveau processus, et
l'interpréteur est neuf : ni mémoire, ni objet HAL, ni socket DDS hérités.

Licence : Apache 2.0, comme tout le dépôt (voir LICENSE).
"""

from __future__ import annotations

import os
import signal
import socket
import subprocess
import sys
import threading
from pathlib import Path
from typing import Any, Dict, Optional

from axis_os.store.runtime.canal_ipc import (
    Booleen, Canal, Chaine, Dictionnaire, Entier, JsonLibre, Liste, Nombre, Objet, UnParmi,
)

RACINE = str(Path(__file__).resolve().parents[3])
HOTE = str(Path(__file__).resolve().with_name("hote_app.py"))

# Les hooks d'une app, plus « fonction » : une fonction de confiance de
# l'OS exécutée sous quotas par SandboxRunner.run(). Pas de « manifest » :
# l'OS lit le manifeste déclaré et signé à côté du code, sans l'exécuter
# (PlayStoreRuntime.install_source).
HOOKS = ("on_start", "run_mission", "on_stop", "fonction")

SCHEMA_APP = Objet({
    "genre": UnParmi("module", "source", "script"),
    "module": Chaine(256),
}, facultatifs={
    "classe": Chaine(256),
    "fonction": Chaine(256),
    "source": Chaine(1 << 20),
    "fichier": Chaine(4096),
    "paquet": Chaine(256),
})

SCHEMA_DEPART = Objet({
    "type": UnParmi("demarrer"),
    "hook": UnParmi(*HOOKS),
    "app": SCHEMA_APP,
    "sys_path": Liste(Chaine(4096), 512),
    "tolerance_lenteur": Nombre(0, 1e6),
}, facultatifs={
    "params": JsonLibre(),
    "etat": Dictionnaire(JsonLibre(), 4096),
    "args": Liste(JsonLibre(), 256),
    "kwargs": Dictionnaire(JsonLibre(), 256),
})


# ─────────────────────────────────────────────────────────────────
#  Comment recharger une app dans le processus neuf
# ─────────────────────────────────────────────────────────────────

class AppNonRechargeable(ValueError):
    """L'app ne peut pas être recréée dans un processus neuf (classe
    locale à une fonction, module sans source…)."""


def spec_depuis_instance(app: Any, source_verifiee: Optional[str] = None) -> Dict[str, Any]:
    """Décrit comment le processus de l'app retrouvera la classe de `app`.

    `source_verifiee` : texte dont la signature a été vérifiée à
    l'installation. Il est alors exécuté tel quel dans le processus de
    l'app (genre « source »), et non relu sur le disque.
    """
    classe = type(app)
    module_nom = classe.__module__
    qualname = classe.__qualname__
    if "<locals>" in qualname:
        raise AppNonRechargeable(
            f"la classe {qualname} est définie dans une fonction : le processus "
            f"de l'app ne peut pas la recréer. Définir la classe au niveau du module.")
    module = sys.modules.get(module_nom)
    paquet = getattr(module, "__package__", None) or None
    fichier = getattr(module, "__file__", None)
    nom_exec = module_nom if module_nom != "__main__" else _nom_script(fichier)

    if module_nom == "__main__":
        # Script lancé directement : seules ses définitions sont reprises
        # (voir hote_app._definitions_seulement), à partir du texte vérifié
        # s'il y en a un.
        if source_verifiee is None:
            if not fichier:
                raise AppNonRechargeable("app définie dans un script sans fichier source")
            source_verifiee = Path(fichier).read_text(encoding="utf-8")
        return {"genre": "script", "module": nom_exec, "classe": qualname,
                "source": source_verifiee, "fichier": fichier or f"<{nom_exec}>",
                "paquet": None}
    if source_verifiee is not None:
        return {"genre": "source", "module": nom_exec, "classe": qualname,
                "source": source_verifiee, "fichier": fichier or f"<{nom_exec}>",
                "paquet": paquet}
    # Module chargé par packaging.py (app chiffrée) : pas de fichier, mais
    # sa source vérifiée a été gardée.
    source_paquet = getattr(module, "__axis_source__", None)
    if source_paquet is not None:
        return {"genre": "source", "module": nom_exec, "classe": qualname,
                "source": source_paquet, "fichier": f"<{nom_exec}>", "paquet": None}
    spec_module = getattr(module, "__spec__", None)
    if spec_module is None or spec_module.name != module_nom:
        raise AppNonRechargeable(f"le module {module_nom} n'est pas importable par son nom")
    return {"genre": "module", "module": module_nom, "classe": qualname}


def _nom_script(fichier: Optional[str]) -> str:
    base = Path(fichier).stem if fichier else "script"
    return "axis_app_" + "".join(c if c.isalnum() else "_" for c in base)


class NonTransmissible(ValueError):
    """Valeur qui n'est pas une donnée transmissible en JSON."""


# Une énumération voyage comme {"$enum": "module:Classe", "$nom": "MEMBRE"} ;
# le processus de l'app la reconstruit (hote_app._decoder_donnee).
ETIQUETTE_ENUM = "$enum"


def encoder_donnee(v: Any, profondeur: int = 0) -> Any:
    """v -> JSON pur, ou NonTransmissible.

    Accepte None, bool, int, float fini, str, list, dict à clés str, et
    les membres d'une énumération définie dans un module importable (ni
    __main__, ni classe locale). Un tuple, un ensemble ou tout autre objet
    est refusé : JSON ne le rendrait pas à l'identique."""
    import enum
    import math
    if profondeur > 20:
        raise NonTransmissible("structure trop profonde")
    if v is None or isinstance(v, (bool, str, int)):
        return v
    if isinstance(v, float):
        if not math.isfinite(v):
            raise NonTransmissible("nombre non fini")
        return v
    if isinstance(v, enum.Enum):
        classe = type(v)
        if classe.__module__ == "__main__" or "<locals>" in classe.__qualname__:
            raise NonTransmissible(f"énumération {classe.__qualname__} non importable")
        return {ETIQUETTE_ENUM: f"{classe.__module__}:{classe.__qualname__}", "$nom": v.name}
    if isinstance(v, list):
        return [encoder_donnee(x, profondeur + 1) for x in v]
    if isinstance(v, dict):
        if ETIQUETTE_ENUM in v:
            raise NonTransmissible("clé réservée $enum")
        sortie = {}
        for k, x in v.items():
            if not isinstance(k, str):
                raise NonTransmissible("clé de dict non textuelle")
            sortie[k] = encoder_donnee(x, profondeur + 1)
        return sortie
    raise NonTransmissible(f"{type(v).__name__} n'est pas une donnée")


def etat_json(app: Any) -> Dict[str, Any]:
    """Attributs de l'instance qui sont des données (voir encoder_donnee).
    Le processus de l'app les réapplique après son propre constructeur ;
    les autres (générateur aléatoire, objets…) sont recréés par le
    constructeur. C'est ce que transmettait pickle, moins tout ce qui
    n'est pas une donnée."""
    try:
        attributs = vars(app)
    except TypeError:
        return {}
    etat = {}
    for k, v in attributs.items():
        if not isinstance(k, str):
            continue
        try:
            etat[k] = encoder_donnee(v)
        except NonTransmissible:
            pass
    return etat


def reference_fonction(fn: Any) -> Dict[str, Any]:
    """Désigne une fonction par son module et son nom, pour que le processus
    neuf la retrouve (SandboxRunner.run). Lève NonTransmissible si elle ne
    peut pas être retrouvée ainsi (objet appelable, fonction locale…)."""
    nom = getattr(fn, "__qualname__", None) or getattr(fn, "__name__", None)
    module_nom = getattr(fn, "__module__", None)
    if module_nom is None and hasattr(fn, "__self__"):
        module_nom = type(fn.__self__).__module__   # méthode intégrée liée
    if not isinstance(nom, str) or not isinstance(module_nom, str) or "<locals>" in nom:
        raise NonTransmissible(f"{fn!r} n'est pas une fonction désignable par son nom")
    module = sys.modules.get(module_nom)
    candidats = [nom, getattr(fn, "__name__", nom)]
    trouve = None
    for c in candidats:
        obj = module
        for partie in c.split("."):
            obj = getattr(obj, partie, None)
        if obj is fn or (obj is not None and obj == fn):
            trouve = c
            break
    if trouve is None:
        raise NonTransmissible(f"{nom} introuvable dans le module {module_nom}")
    if module_nom == "__main__":
        fichier = getattr(module, "__file__", None)
        if not fichier:
            raise NonTransmissible("fonction définie dans un script sans fichier")
        return {"genre": "script", "module": _nom_script(fichier), "fonction": trouve,
                "source": Path(fichier).read_text(encoding="utf-8"), "fichier": fichier}
    return {"genre": "module", "module": module_nom, "fonction": trouve}


# ─────────────────────────────────────────────────────────────────
#  Le processus lui-même
# ─────────────────────────────────────────────────────────────────

def environnement_minimal() -> Dict[str, str]:
    """PATH et langue seulement. Le réglage de lenteur du pont
    (AXIS_OS_BRIDGE_TOLERANCE) passe par le message de départ."""
    return {"PATH": "/usr/local/bin:/usr/bin:/bin", "LANG": "C.UTF-8", "LC_ALL": "C.UTF-8"}


def _affichable(texte: str, n: int = 500) -> str:
    """Texte venu de l'app, rendu sûr pour le terminal de l'OS : tronqué
    AVANT d'être parcouru (une ligne énorme ne doit rien coûter), puis
    caractères de contrôle remplacés par « ? »."""
    coupe = len(texte) > n
    t = "".join(c if c.isprintable() else "?" for c in texte[:n])
    return t[:n - 1] + "…" if coupe else t


_PROCESSUS_NON_INSPECTABLE: Optional[bool] = None


def rendre_os_non_inspectable() -> bool:
    """Interdit aux processus du même utilisateur, donc à ceux des apps,
    d'inspecter le processus de l'OS : lecture de son environnement et de
    sa mémoire par /proc, attachement par ptrace (prctl PR_SET_DUMPABLE à
    0, Linux). Effet de bord : plus de vidage mémoire (core dump) de l'OS.
    Fait une fois par processus ; rend True si le réglage est en place.

    Le processus d'une app, lui, reste inspectable par l'OS : le réglage
    n'est pas transmis à un programme lancé par exec."""
    global _PROCESSUS_NON_INSPECTABLE
    if _PROCESSUS_NON_INSPECTABLE is None:
        import ctypes
        try:
            libc = ctypes.CDLL(None, use_errno=True)
            libc.prctl(4, 0, 0, 0, 0)               # 4 = PR_SET_DUMPABLE
            _PROCESSUS_NON_INSPECTABLE = libc.prctl(3, 0, 0, 0, 0) == 0   # 3 = PR_GET_DUMPABLE
        except (OSError, AttributeError):
            _PROCESSUS_NON_INSPECTABLE = False
    return _PROCESSUS_NON_INSPECTABLE


class ProcessusHote:
    """Le processus d'une app, vu par l'OS. Offre is_alive / join /
    terminate / kill / pid, comme multiprocessing.Process, pour que la
    surveillance des quotas de SandboxRunner s'applique telle quelle.

    Le processus n'est récolté (wait) qu'à la toute fin, dans fermer() :
    tant qu'il ne l'est pas, son numéro reste réservé, et le signal envoyé
    à son groupe ne peut pas atteindre un autre groupe qui aurait reçu le
    même numéro."""

    MAX_LIGNES_RELAYEES = 5000
    MAX_OCTETS_RELAYES = 1 << 20      # par appel ; au-delà, lu et jeté
    MAX_LIGNE = 2000                  # octets gardés d'une ligne

    def __init__(self, app_id: str):
        self.app_id = app_id
        rendre_os_non_inspectable()
        parent, enfant = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            fd = enfant.fileno()
            self._popen = subprocess.Popen(
                [sys.executable, "-I", HOTE, str(fd), RACINE, str(os.getpid())],
                pass_fds=(fd,), close_fds=True, start_new_session=True,
                stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                env=environnement_minimal(), cwd=os.getcwd(),
            )
        except Exception:
            parent.close()
            raise
        finally:
            enfant.close()
        self.canal = Canal(parent)
        self._sortie = self._popen.stdout.fileno()
        self._arret_relais = threading.Event()
        self._eof_vu = False
        self._erreur_relais = False
        self._relais = threading.Thread(target=self._relayer, daemon=True,
                                        name=f"sortie-app-{app_id}")
        self._relais.start()

    # ── Sortie de l'app ──────────────────────────────────────────────

    def _afficher(self, octets: bytes) -> None:
        ligne = octets[:self.MAX_LIGNE].decode("utf-8", "replace").rstrip("\r")
        print(f"    [app {_affichable(self.app_id, 80)}] {_affichable(ligne)}")

    def _relayer(self) -> None:
        """Réaffiche la sortie de l'app, ligne par ligne, neutralisée et
        bornée : au plus MAX_LIGNE octets gardés par ligne, MAX_LIGNES et
        MAX_OCTETS par appel, le reste est lu et jeté. Lecture par morceaux
        sur le descripteur brut, avec un délai (poll, sans limite sur le
        numéro du descripteur) : le thread s'arrête sur ordre (fermer) même
        si un processus lancé par l'app garde le tube ouvert."""
        import select
        scrutateur = select.poll()
        scrutateur.register(self._sortie, select.POLLIN | select.POLLHUP | select.POLLERR)
        tampon, lignes, total, ignorer, plein = b"", 0, 0, False, False
        while not self._arret_relais.is_set():
            try:
                if not scrutateur.poll(100):
                    continue
                bloc = os.read(self._sortie, 65536)
            except (OSError, ValueError):
                self._erreur_relais = True
                break
            if not bloc:
                self._eof_vu = True
                break
            total += len(bloc)
            if plein:
                continue                      # lu et jeté
            if total > self.MAX_OCTETS_RELAYES:
                plein = True
                print(f"    [app {_affichable(self.app_id, 80)}] … sortie tronquée")
                continue
            morceaux = (tampon + bloc).split(b"\n")
            tampon = morceaux.pop()           # dernière ligne, pas encore finie
            for ligne in morceaux:
                if not ignorer:
                    self._afficher(ligne)
                    lignes += 1
                    if lignes >= self.MAX_LIGNES_RELAYEES:
                        plein = True
                        print(f"    [app {_affichable(self.app_id, 80)}] … sortie tronquée")
                        break
                ignorer = False
            if not plein and len(tampon) > self.MAX_LIGNE:
                # Ligne trop longue : son début est affiché, la suite
                # ignorée jusqu'au prochain saut de ligne.
                if not ignorer:
                    self._afficher(tampon)
                    lignes += 1
                tampon, ignorer = b"", True
        if tampon and not ignorer and not plein:
            self._afficher(tampon)

    # ── Cycle de vie ─────────────────────────────────────────────────

    @property
    def pid(self) -> int:
        return self._popen.pid

    def is_alive(self) -> bool:
        """Vrai tant que l'hôte n'est pas terminé, SANS le récolter (WNOWAIT) :
        voir la docstring de la classe."""
        if self._popen.returncode is not None:
            return False
        try:
            info = os.waitid(os.P_PID, self._popen.pid,
                             os.WEXITED | os.WNOHANG | os.WNOWAIT)
        except ChildProcessError:
            return False
        return info is None

    def join(self, timeout: Optional[float] = None) -> None:
        import time
        fin = None if timeout is None else time.monotonic() + timeout
        while self.is_alive() and (fin is None or time.monotonic() < fin):
            time.sleep(0.01)

    def _signal_groupe(self, sig: int) -> None:
        """Envoie `sig` au groupe de processus de l'app (l'hôte est chef de
        son groupe, start_new_session) : un processus que l'app aurait lancé
        sans changer de groupe est arrêté avec elle. Un processus qui a
        quitté le groupe (setsid) n'est pas atteint : voir fermer()."""
        if self._popen.returncode is not None:
            return                    # déjà récolté : numéro peut-être réutilisé
        try:
            os.killpg(self._popen.pid, sig)
        except (ProcessLookupError, PermissionError):
            pass

    def terminate(self) -> None:
        self._signal_groupe(signal.SIGTERM)

    def kill(self) -> None:
        self._signal_groupe(signal.SIGKILL)

    def fermer(self) -> None:
        """Tue l'hôte et son groupe, puis seulement le récolte ; ferme le
        pont ; arrête le relais de sortie et ferme le tube. Ne bloque
        jamais plus de quelques secondes, même si un processus lancé par
        l'app a quitté le groupe et garde le tube ouvert."""
        self.kill()
        try:
            self._popen.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            pass
        self.canal.fermer()
        self._relais.join(timeout=1.0)
        self._arret_relais.set()
        self._relais.join(timeout=1.0)
        if not self._eof_vu and not self._erreur_relais:
            print(f"  [SECURITY] app '{_affichable(self.app_id, 80)}' : un processus "
                  f"qu'elle a lancé a quitté son groupe et garde sa sortie ouverte ; "
                  f"il n'a pas pu être arrêté (voir docs/isolation_apps.md, étape 4).")
        if not self._relais.is_alive():
            try:
                self._popen.stdout.close()
            except OSError:
                pass


def message_depart(hook: str, spec: Dict[str, Any], params: Any = None,
                   etat: Optional[Dict[str, Any]] = None,
                   tolerance_lenteur: float = 60.0, args: Optional[list] = None,
                   kwargs: Optional[dict] = None) -> Dict[str, Any]:
    return {"type": "demarrer", "hook": hook, "app": spec, "params": params,
            "etat": etat, "args": args, "kwargs": kwargs,
            "sys_path": [p for p in sys.path if isinstance(p, str) and p],
            "tolerance_lenteur": float(tolerance_lenteur)}


# ─────────────────────────────────────────────────────────────────
#  Ce que l'OS accepte en retour
# ─────────────────────────────────────────────────────────────────

_JOURNAL = Liste(Chaine(8192), 10_000)

SCHEMA_MISSION = Objet({"success": Booleen()}, facultatifs={
    "items_processed": Entier(0, 10 ** 12),
    "error_message": Chaine(8192),
    "metrics": JsonLibre(),
    "log": _JOURNAL,
})

SCHEMA_RESULTAT = {
    "on_start": Objet({"pret": Booleen(), "log": _JOURNAL}),
    "on_stop": Objet({"ok": Booleen(), "log": _JOURNAL}),
    "run_mission": Objet({}, facultatifs={"mission": SCHEMA_MISSION}),
    "fonction": Objet({}, facultatifs={"valeur": JsonLibre()}),
}

SCHEMA_FIN = Objet({"type": UnParmi("fin")},
                   facultatifs={"resultat": JsonLibre(), "erreur": Chaine(4000)})
