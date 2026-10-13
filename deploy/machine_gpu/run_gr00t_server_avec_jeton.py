"""
Lance le serveur GR00T de NVIDIA (gr00t/eval/run_gr00t_server.py) en
exigeant un jeton d'API, lu dans la variable d'environnement GR00T_API_TOKEN.

GR00T est le modèle de NVIDIA pour robots humanoïdes que nous affinons comme
VLA (modèle « vision-langage-action » : il regarde par les caméras et
propose des mouvements). Le serveur charge le modèle et répond aux demandes
d'action du client, ici le test de simulation lancé par evaluer_vla.sh. Le
jeton d'API est un secret que chaque requête doit présenter : il empêche
les autres processus de la machine d'interroger le serveur.

Pourquoi un lanceur (relevé lors d'un audit de sécurité du dépôt, le
2026-10-10) : run_gr00t_server.py n'a PAS d'option --api-token. Sa
ServerConfig (la liste de ses options) ne la déclare pas ; tyro, la
bibliothèque qui lit les options de la ligne de commande, refuserait donc
l'option, et le serveur ne démarrerait pas. La classe PolicyServer, elle,
accepte api_token et refuse alors toute requête sans le bon jeton. Ce
lanceur transmet le jeton à PolicyServer SANS le mettre sur une ligne de
commande : il serait sinon visible dans « ps » (la liste des processus)
pendant toute la vie du serveur.

Le nom commence par « run_gr00t_server » exprès : evaluer_vla.sh suit et
arrête le serveur par « pgrep -f run_gr00t_server » et
« pkill -f run_gr00t_server », qui cherchent les processus par leur nom.

Usage, dans ~/Isaac-GR00T (l'environnement Python de GR00T), avec les mêmes
options que run_gr00t_server.py :

    GR00T_API_TOKEN=<jeton> uv run python <ce fichier> --model-path … --port 5555

Limite connue : le lanceur dépend de noms internes au code de NVIDIA
(PolicyServer, importé dans le module run_gr00t_server, son paramètre
api_token, ainsi que la fonction main et la classe ServerConfig de ce
module). S'ils changent, le lanceur s'arrête avec un message au lieu de
démarrer un serveur SANS jeton. Le cas d'un PolicyServer sans paramètre
api_token est vérifié par test/test_lanceur_gr00t_jeton.py.
"""
import inspect
import os
import sys


def main() -> int:
    jeton = os.environ.get("GR00T_API_TOKEN", "")
    if not jeton:
        print("ÉCHEC : GR00T_API_TOKEN absent — sans jeton, lancer "
              "gr00t/eval/run_gr00t_server.py directement.", file=sys.stderr)
        return 2
    import tyro
    import gr00t.eval.run_gr00t_server as serveur

    classe = getattr(serveur, "PolicyServer", None)
    if classe is None or not callable(getattr(serveur, "main", None)) \
            or not hasattr(serveur, "ServerConfig") \
            or "api_token" not in inspect.signature(classe).parameters:
        # Jamais de repli silencieux vers un serveur sans jeton : si le jeton
        # ne peut pas être imposé, le lanceur refuse de démarrer.
        print("ÉCHEC : cette version de gr00t/eval/run_gr00t_server.py ne permet pas "
              "de passer un jeton (PolicyServer(api_token=…) introuvable) — mettre à "
              "jour ce lanceur.", file=sys.stderr)
        return 3
    # main() construit « PolicyServer(policy=…, host=…, port=…) » en lisant
    # le nom PolicyServer dans son propre module au moment de l'appel. On
    # remplace donc ce nom, dans le module, par une sous-classe qui IMPOSE le
    # jeton, quoi que main() lui passe.
    # Pourquoi pas functools.partial (relevé par le même audit) : si une
    # future version de main() passait elle-même api_token=None, le mot-clé
    # de l'appel l'emporterait sur celui de la partial, et le serveur
    # démarrerait sans jeton tout en affichant « exigé ». Ce cas est vérifié
    # par test/test_lanceur_gr00t_jeton.py.
    class _PolicyServerAvecJeton(classe):
        def __init__(self, *args, **kwargs):
            kwargs["api_token"] = jeton
            super().__init__(*args, **kwargs)

    serveur.PolicyServer = _PolicyServerAvecJeton
    print("Jeton d'API : exigé par le serveur.", flush=True)
    serveur.main(tyro.cli(serveur.ServerConfig))
    return 0


if __name__ == "__main__":
    sys.exit(main())
