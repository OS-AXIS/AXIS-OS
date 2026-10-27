"""
AXIS-OS CLI — Templates
=========================
Modèles (templates) des fichiers générés par `axis-os init <app>` :
app.py (le code de l'app), manifest.json (ses métadonnées pour le Play
Store, la boutique d'apps d'AXIS-OS), test_app.py (un test minimal) et
README.md.

Chaque modèle est rempli par str.format() dans cmd_init (main.py), avec
les champs app_name, app_id, class_name, author, author_email et
app_id_short. Piège à connaître avant d'en modifier un : les accolades
qui doivent apparaître telles quelles dans le fichier généré sont
doublées ({{ et }}), sinon format() les prend pour un champ à remplir.

Le manifeste (la fiche qui décrit l'app : identifiant, version,
permissions, matériel requis) est déclaré dans le code de app.py, et
c'est lui que lit `axis-os simulate`. manifest.json en reprend une
partie ; c'est ce fichier que vérifie `axis-os publish`. Le manifeste
généré part de réglages prudents : seule la permission ACTUATORS_READ
(lecture de l'état des articulations, accordée sans confirmation),
vitesse de chaque commande articulaire plafonnée à 50 %
(max_velocity_ratio=0.5) et supervision humaine déclarée
(requires_human_supervision=True). Cette dernière n'est pour l'instant
qu'une déclaration : le runtime (PlayStoreRuntime, qui installe et
exécute les apps) ne s'en sert pas.

Le test généré tourne sur un robot simulé. Il accorde donc sans
confirmation, via accepter_tout_SIMULATION, les permissions DANGEROUS
(celles qui exigent normalement l'accord explicite d'un humain, comme
commander les moteurs) ; sur un vrai robot, cet accord reste exigé.

Licence : Apache 2.0
"""

APP_PY_TEMPLATE = '''"""
{app_name} — AXIS-OS Play Store App
=====================================
Généré par `axis-os init`.
"""

from axis_os.store.sdk.manifest import (
    AppManifest, Permission, AppCategory,
    PricingModel, HardwareRequirements
)
from axis_os.store.sdk.app_base import RobotApp, RobotContext, MissionResult

# Nombre d'étapes de la mission d'exemple.
STEPS = 5


class {class_name}(RobotApp):

    def manifest(self) -> AppManifest:
        return AppManifest(
            app_id="{app_id}",
            name="{app_name}",
            version="0.1.0",
            author="{author}",
            author_email="{author_email}",
            description="Décrivez votre app ici.",
            category=AppCategory.UTILITIES,
            permissions=[
                Permission.ACTUATORS_READ,
                Permission.SYSTEM_INFO,        # has_capability, robot_info
                Permission.MISSION_LIFECYCLE,  # should_stop, report_progress
                # Ajoutez les permissions dont votre app a besoin (voir
                # docs/API_REFERENCE.md : chaque API dit la sienne).
            ],
            hardware=HardwareRequirements(min_dof=0),
            # Version de l'API du SDK dont l'app a besoin (ctx.api_version).
            min_sdk_version="1.0.0",
            pricing_model=PricingModel.FREE,
            max_velocity_ratio=0.5,
            requires_human_supervision=True,
        )

    def on_start(self, ctx: RobotContext) -> bool:
        ctx.log("Initialisation...")
        return True

    def run_mission(self, ctx: RobotContext, params: dict) -> MissionResult:
        ctx.log("Mission démarrée")
        # Demander au robot ce qu'il sait faire, plutôt que de le supposer.
        if not ctx.has_capability("arms"):
            return MissionResult(success=False, mission_id="", app_id="",
                                 duration_s=0.0, error_message="ce robot n'a pas de bras")

        done = 0
        for step in range(STEPS):
            # Un humain ou l'OS peut demander l'arrêt : on range et on rend
            # la main (l'arrêt d'urgence, lui, coupe tout de toute façon).
            if ctx.should_stop():
                break
            # ── Votre logique métier ici ──────────────────────
            done += 1
            ctx.report_progress(done / STEPS, f"étape {{done}}/{{STEPS}}")
            ctx.sleep(0.1)

        return MissionResult(
            success=done == STEPS,
            mission_id="",
            app_id=self.manifest().app_id,
            duration_s=0.0,
            items_processed=done,
        )

    def on_stop(self, ctx: RobotContext) -> None:
        ctx.log("Arrêt propre")
'''


MANIFEST_JSON_TEMPLATE = '''{{
  "app_id": "{app_id}",
  "name": "{app_name}",
  "version": "0.1.0",
  "author": "{author}",
  "author_email": "{author_email}",
  "category": "utilities",
  "pricing_model": "free",
  "base_price_eur": 0.0
}}
'''


TEST_APP_TEMPLATE = '''"""
Tests pour {app_name}
"""

import sys
sys.path.insert(0, ".")

from axis_os.testing import AppTestHarness
from app import {class_name}


def test_app():
    # Robot simulé léger (axis_os.testing) : l'app tourne comme sur un
    # robot, dans son processus, sous ses permissions. En SIMULATION, les
    # permissions DANGEROUS sont accordées sans humain ; sur un vrai
    # robot, elles exigent sa confirmation.
    with AppTestHarness({class_name}()) as h:
        result = h.run({{}})
        assert result is not None and result.success, result
        assert h.runtime.get_progress("{app_id}").fraction == 1.0
    print("✓ Test passé")


if __name__ == "__main__":
    test_app()
'''


README_APP_TEMPLATE = '''# {app_name}

App AXIS-OS générée par `axis-os init`.

## Développement

```bash
python test_app.py   # teste sur le robot simulé léger
axis validate        # vérifie l'app comme l'OS le fera
axis simulate        # mission sur le G1 simulé
axis publish --dry-run
```

## Structure

```
{app_id_short}/
├── app.py           # logique de votre app
├── manifest.json    # métadonnées Play Store
├── test_app.py       # tests
└── README.md
```
'''
