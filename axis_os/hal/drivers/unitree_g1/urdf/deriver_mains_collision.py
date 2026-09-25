"""
Dérive, de l'URDF OFFICIEL du G1 29 DOF, une copie où les mains en
caoutchouc ont une géométrie de COLLISION.

POURQUOI : dans g1_29dof_rev_1_0.urdf (unitree_ros), les liens
`left_rubber_hand` et `right_rubber_hand` ont un visuel mais AUCUNE
collision. Pour un simulateur, ce sont des fantômes : on les voit, rien
ne les arrête. Constaté au test W3 : les mains TRAVERSAIENT le carton au
lieu de le serrer. Dans l'URDF 23 DOF, la main fait partie du poignet et
possède bien sa collision — l'omission est propre au fichier 29 DOF.

CE QUE FAIT CE SCRIPT, et rien d'autre : pour chaque main sans
collision, il ajoute une collision identique à son visuel (même maillage,
même origine). Le simulateur en tire une enveloppe convexe. Aucune autre
valeur n'est touchée — ni masses, ni limites, ni axes.

L'URDF OFFICIEL N'EST PAS MODIFIÉ : il reste la source de vérité
d'AXIS-OS. La copie dérivée est un artefact de SIMULATION, régénérable,
et le vrai robot n'en a pas besoin — ses mains ont un volume physique.

    python3 deriver_mains_collision.py
      -> g1_29dof_rev_1_0_mains_collision.urdf
"""
import copy
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ICI = Path(__file__).parent
SOURCE = ICI / "g1_29dof_rev_1_0.urdf"
CIBLE = ICI / "g1_29dof_rev_1_0_mains_collision.urdf"
MAINS = ("left_rubber_hand", "right_rubber_hand")


def deriver(source: Path = SOURCE, cible: Path = CIBLE) -> list:
    arbre = ET.parse(source)
    ajoutees = []
    for lien in arbre.getroot().findall("link"):
        if lien.get("name") not in MAINS:
            continue
        if lien.find("collision") is not None:
            continue                      # déjà présente : on ne touche à rien
        visuel = lien.find("visual")
        if visuel is None:
            raise ValueError(f"{lien.get('name')} : ni collision ni visuel")
        col = ET.SubElement(lien, "collision")
        for enfant in ("origin", "geometry"):
            el = visuel.find(enfant)
            if el is not None:
                col.append(copy.deepcopy(el))
        ajoutees.append(lien.get("name"))
    arbre.write(cible, encoding="utf-8", xml_declaration=True)
    return ajoutees


if __name__ == "__main__":
    a = deriver()
    print(f"collision ajoutée à : {a or 'aucune (déjà présentes)'} -> {CIBLE.name}")
    sys.exit(0)
