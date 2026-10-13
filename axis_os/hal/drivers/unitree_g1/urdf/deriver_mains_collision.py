"""
Dérive, de l'URDF officiel du G1 29 DOF (29 degrés de liberté,
c'est-à-dire 29 articulations motorisées), une copie où les mains en
caoutchouc ont une géométrie de collision.

Un URDF (Unified Robot Description Format) est le fichier XML standard
qui décrit un robot. Chaque « lien » (pièce rigide) y a un visuel, la
forme affichée, et éventuellement une collision, la forme dont le
simulateur se sert pour calculer les contacts.

Pourquoi ce script : dans g1_29dof_rev_1_0.urdf (dépôt officiel
unitree_ros), les liens `left_rubber_hand` et `right_rubber_hand` ont un
visuel mais aucune collision. Pour un simulateur, ce sont des fantômes :
on les voit, mais rien ne les arrête. Constaté en simulation, lors de
l'essai de prise d'un carton posé sur une table : les mains traversaient
le carton au lieu de le serrer (voir docs/validation_port_carton.md).
Dans l'URDF 23 DOF, la main fait partie du poignet et possède bien sa
collision : l'omission est propre au fichier 29 DOF.

Ce que fait ce script, et rien d'autre : pour chaque main sans
collision, il ajoute une collision identique à son visuel (même maillage
3D, même origine). Le simulateur en tire une enveloppe convexe (la plus
petite forme sans creux qui contient le maillage). Aucune autre valeur
n'est touchée : ni masses, ni limites, ni axes.

L'URDF officiel n'est pas modifié : il reste la source de vérité
d'AXIS-OS. La copie dérivée est un fichier produit pour la simulation
seulement, que l'on peut régénérer à tout moment ; le vrai robot n'en a
pas besoin, puisque ses mains ont un volume physique.

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
    """Écrit la copie dérivée dans `cible` et renvoie la liste des mains
    auxquelles une collision a été ajoutée (vide si toutes en avaient
    déjà une). Lève ValueError si une main n'a ni collision ni visuel."""
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
