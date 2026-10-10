"""
Lance le serveur GR00T de NVIDIA (gr00t/eval/run_gr00t_server.py) AVEC un
jeton d'API, lu dans la variable d'environnement GR00T_API_TOKEN.

POURQUOI UN LANCEUR (revue de l'audit du 2026-10-10) : run_gr00t_server.py
n'a PAS d'option --api-token — sa ServerConfig ne la déclare pas, tyro
refuserait l'option et le serveur ne démarrerait pas —, alors que la classe
PolicyServer, elle, accepte api_token et refuse toute requête sans le bon
jeton. Ce lanceur transmet le jeton à PolicyServer SANS le mettre sur une
ligne de commande : il serait sinon visible dans « ps » pendant toute la
vie du serveur.

Nom en « run_gr00t_server… » VOULU : evaluer_vla.sh suit et arrête le
serveur par « pgrep -f run_gr00t_server » et « pkill -f run_gr00t_server ».

Usage, dans ~/Isaac-GR00T (environnement de GR00T), mêmes options que
run_gr00t_server.py :

    GR00T_API_TOKEN=<jeton> uv run python <ce fichier> --model-path … --port 5555

Dépend de deux noms internes de NVIDIA (PolicyServer importé dans le module
run_gr00t_server, et sa fonction main) : s'ils changent, le lanceur
s'arrête avec un message au lieu de démarrer un serveur SANS jeton.
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
        # Jamais de repli silencieux vers un serveur sans jeton.
        print("ÉCHEC : cette version de gr00t/eval/run_gr00t_server.py ne permet pas "
              "de passer un jeton (PolicyServer(api_token=…) introuvable) — mettre à "
              "jour ce lanceur.", file=sys.stderr)
        return 3
    # main() construit « PolicyServer(policy=…, host=…, port=…) » par le nom
    # du module : on y substitue une sous-classe qui IMPOSE le jeton, quoi que
    # main() lui passe. Pas functools.partial (revue de l'audit) : si une
    # future version de main() passait elle-même api_token=None, le mot-clé
    # de l'appel l'emporterait sur celui de la partial, et le serveur
    # démarrerait sans jeton en affichant « exigé ».
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
