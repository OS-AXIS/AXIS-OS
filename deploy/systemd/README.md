# Déploiement AXIS-OS via systemd

`axis-os.service` fournit un chien de garde système (`WatchdogSec=2s`)
— voir sa justification complète dans le fichier lui-même, et
`axis_os/hal/watchdog/system.py::_sd_notify_watchdog()` pour le code
côté application qui envoie le heartbeat.

## Installation

```bash
# Créer l'utilisateur système dédié (jamais lancer AXIS-OS en root)
sudo useradd --system --no-create-home axis-os

# Copier le code vers le chemin attendu par ExecStart (à adapter si
# différent)
sudo mkdir -p /opt/axis-os
sudo cp -r . /opt/axis-os/
sudo chown -R axis-os:axis-os /opt/axis-os

# Installer le service
sudo cp deploy/systemd/axis-os.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now axis-os.service
```

## Vérification

```bash
systemctl status axis-os.service
journalctl -u axis-os.service -f
```

Le chien de garde est actif dès que `systemctl status` affiche
`Watchdog: enabled` — sinon, vérifier que `Type=notify` est bien pris
en compte et qu'AXIS-OS tourne dans un contexte où `NOTIFY_SOCKET`
est défini par systemd (c'est automatique sous `Type=notify`, rien à
configurer côté application).

## Limite connue

`ExecStart=/opt/axis-os/venv/bin/python -m axis_os.main` suppose un
point d'entrée `axis_os/main.py` qui n'existe pas encore dans ce
repo — AXIS-OS est aujourd'hui utilisé comme bibliothèque (voir les
apps de référence et les tests), pas encore comme service autonome.
Ce fichier unit est prêt pour le jour où ce point d'entrée existera ;
en attendant, adapter `ExecStart` au script réel utilisé pour lancer
`PlayStoreRuntime` en production.
