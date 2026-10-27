"""
AXIS-OS — Version de l'API du SDK des apps
==========================================
API_VERSION est la version de l'interface qu'une app voit (RobotContext,
ses méthodes, leurs codes de refus), au format SemVer
(MAJEUR.MINEUR.CORRECTIF). Elle est distincte de la version d'AXIS-OS
(axis_os.__version__) : l'OS peut changer sans que l'API des apps change.

  - MAJEUR : changement cassant pour une app existante ;
  - MINEUR : ajout compatible (nouvelle méthode, nouveau code) ;
  - CORRECTIF : correction sans changement d'interface.

Licence : Apache 2.0 (open-source — SDK public)
"""

API_VERSION = "1.0.0"
