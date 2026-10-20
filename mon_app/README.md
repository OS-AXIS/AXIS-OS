# Mon App

App AXIS-OS d'exemple, générée par l'outil en ligne de commande d'AXIS-OS
(`python3 -m axis_os.cli.main init`). Le guide complet est
[docs/CREER_UNE_APP.md](../docs/CREER_UNE_APP.md).

## Développement

Le dépôt ne fournit pas encore de commande `axis-os` installable : l'outil
se lance avec `python3 -m axis_os.cli.main`, depuis le dossier de l'app.

```bash
# depuis la racine du dépôt AXIS-OS
export PYTHONPATH="$PWD"     # pour que « import axis_os » fonctionne
cd mon_app

# teste l'app sur un robot virtuel (physique simplifiée, sans Isaac Sim)
python3 -m axis_os.cli.main simulate

# vérifie le manifeste ; l'envoi au Play Store n'est pas encore branché
python3 -m axis_os.cli.main publish --dry-run
```

## Structure

```
mon_app/
├── app.py           # logique de votre app
├── manifest.json    # métadonnées Play Store
├── test_app.py      # tests
└── README.md
```
