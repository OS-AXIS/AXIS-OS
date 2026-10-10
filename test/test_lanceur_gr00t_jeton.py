"""
Lanceur du serveur GR00T avec jeton d'API (revue de l'audit du 2026-10-10).

run_gr00t_server.py de NVIDIA n'a pas d'option --api-token : le lanceur
deploy/machine_gpu/run_gr00t_server_avec_jeton.py lit GR00T_API_TOKEN et le
passe à PolicyServer. Vérifié ici contre de FAUX modules gr00t et tyro qui
reproduisent la structure du fichier de NVIDIA (ServerConfig, main(config)
qui construit PolicyServer(policy=…, host=…, port=…)) :
  - le jeton arrive à PolicyServer, les options passent telles quelles ;
  - sans la variable : arrêt, pas de serveur ;
  - PolicyServer sans paramètre api_token : arrêt, jamais un serveur sans
    jeton ;
  - le nom du lanceur reste trouvable par « pgrep -f run_gr00t_server » ;
  - le jeton ne passe sur aucune ligne de commande des scripts, ni par
    sudo --preserve-env (qui l'écrit dans son journal : vérifié).
"""
import json
import os
import subprocess
import sys
import tempfile
import textwrap
from pathlib import Path

RACINE = Path(__file__).resolve().parents[1]
LANCEUR = RACINE / "deploy/machine_gpu/run_gr00t_server_avec_jeton.py"

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def faux_gr00t(dossier: Path, avec_api_token: bool = True,
               main_passe_none: bool = False) -> Path:
    (dossier / "gr00t" / "eval").mkdir(parents=True)
    (dossier / "gr00t" / "__init__.py").write_text("")
    (dossier / "gr00t" / "eval" / "__init__.py").write_text("")
    param = ", api_token=None" if avec_api_token else ""
    garde = "self.api_token = api_token" if avec_api_token else "self.api_token = None"
    (dossier / "gr00t" / "eval" / "run_gr00t_server.py").write_text(textwrap.dedent(f"""
        import json, os
        from dataclasses import dataclass

        class PolicyServer:
            def __init__(self, policy, host="*", port=5555{param}):
                self.host, self.port = host, port
                {garde}
            def __enter__(self):
                return self
            def __exit__(self, *a):
                return False
            def run(self):
                with open(os.environ["SORTIE_ESSAI"], "w") as f:
                    json.dump({{"api_token": self.api_token, "host": self.host,
                               "port": self.port}}, f)

        @dataclass
        class ServerConfig:
            model_path: str = None
            host: str = "0.0.0.0"
            port: int = 5555

        def main(config):
            kw = {{MAIN_KW}}
            with PolicyServer(policy=object(), host=config.host, port=config.port, **kw) as s:
                s.run()
    """).replace("{MAIN_KW}", "{'api_token': None}" if main_passe_none else "{}"))
    (dossier / "tyro.py").write_text(textwrap.dedent("""
        import sys
        def cli(cls):
            a, kw = sys.argv[1:], {}
            for i in range(0, len(a), 2):
                cle = a[i][2:].replace("-", "_")
                kw[cle] = int(a[i + 1]) if cle == "port" else a[i + 1]
            return cls(**kw)
    """))
    return dossier


def lancer(dossier: Path, jeton):
    sortie = dossier / "sortie.json"
    env = {k: v for k, v in os.environ.items() if k != "GR00T_API_TOKEN"}
    env.update(PYTHONPATH=str(dossier), SORTIE_ESSAI=str(sortie))
    if jeton is not None:
        env["GR00T_API_TOKEN"] = jeton
    r = subprocess.run([sys.executable, str(LANCEUR), "--model-path", "/m",
                        "--host", "127.0.0.1", "--port", "5599"],
                       env=env, capture_output=True, text=True, timeout=60)
    lu = json.loads(sortie.read_text()) if sortie.exists() else None
    return r, lu


print("\n  Lanceur GR00T avec jeton\n")
d = faux_gr00t(Path(tempfile.mkdtemp()))
r, lu = lancer(d, "jeton-de-test")
check(f"Le jeton arrive à PolicyServer ({lu})", r.returncode == 0 and lu is not None
      and lu["api_token"] == "jeton-de-test")
check("Les options passent telles quelles (hôte 127.0.0.1, port 5599)",
      lu is not None and lu["host"] == "127.0.0.1" and lu["port"] == 5599)
check("Le jeton n'est pas affiché par le lanceur", "jeton-de-test" not in r.stdout + r.stderr)

d = faux_gr00t(Path(tempfile.mkdtemp()))
r, lu = lancer(d, None)
check("Sans GR00T_API_TOKEN : arrêt (code 2), aucun serveur", r.returncode == 2 and lu is None)

d = faux_gr00t(Path(tempfile.mkdtemp()), avec_api_token=False)
r, lu = lancer(d, "jeton-de-test")
check("PolicyServer sans api_token : arrêt (code 3), JAMAIS un serveur sans jeton",
      r.returncode == 3 and lu is None)

# Trouvé à la revue : une future version de main() qui passerait elle-même
# api_token=None ne doit pas faire démarrer un serveur sans jeton.
d = faux_gr00t(Path(tempfile.mkdtemp()), main_passe_none=True)
r, lu = lancer(d, "jeton-de-test")
check(f"main() de NVIDIA qui passerait api_token=None : le jeton est IMPOSÉ quand même ({lu})",
      r.returncode == 0 and lu is not None and lu["api_token"] == "jeton-de-test")

check("Nom trouvable par « pgrep -f run_gr00t_server » (suivi et arrêt du serveur)",
      "run_gr00t_server" in LANCEUR.name)

EVALUER = (RACINE / "deploy/machine_gpu/evaluer_vla.sh").read_text(encoding="utf-8")
LANCER_T1 = (RACINE / "deploy/machine_gpu/lancer_t1.sh").read_text(encoding="utf-8")
code = lambda t: "\n".join(l for l in t.splitlines() if not l.lstrip().startswith("#"))
check("evaluer_vla.sh : plus d'option --api-token (inconnue du serveur de NVIDIA)",
      "--api-token" not in code(EVALUER))
check("evaluer_vla.sh : le lanceur est utilisé quand le jeton est donné",
      "run_gr00t_server_avec_jeton.py" in EVALUER)
check("lancer_t1.sh : jeton transmis par l'entrée standard (docker exec --env-file /dev/stdin)",
      "--env-file /dev/stdin" in code(LANCER_T1)
      and "printf 'GR00T_API_TOKEN=%s\\n' \"$GR00T_API_TOKEN\"" in code(LANCER_T1))
check("lancer_t1.sh : ni « -e GR00T_API_TOKEN… » ni « --preserve-env » (journal de sudo, ps)",
      "-e GR00T_API_TOKEN" not in code(LANCER_T1) and "--preserve-env" not in code(LANCER_T1)
      and "GR00T_API_TOKEN=$" not in code(LANCER_T1))
check("lancer_t1.sh : sans jeton, la commande d'origine est inchangée",
      'sudo docker exec -d "$CONTENEUR" bash -c "$COMMANDE"' in LANCER_T1)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
