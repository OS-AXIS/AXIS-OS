"""
Couleur des colis : l'IA trouve les boîtes, les pixels disent leur couleur.

Pourquoi ce partage des rôles : OWL-ViT (un modèle de détection d'objets
« à vocabulaire ouvert », qui trouve dans une image les objets décrits par
une courte phrase) reconnaît bien « une boîte ». En revanche, personne n'a
jamais vérifié qu'il distingue de façon fiable « une boîte rouge » d'« une
boîte bleue ». Plutôt que de parier là-dessus, chaque détection est gardée
ou écartée selon la teinte mesurée dans les pixels, au centre de sa boîte
englobante (le rectangle qui entoure l'objet dans l'image). C'est simple,
déterministe, testable sans GPU (carte graphique), et l'on sait qui fait
quoi : l'IA localise les boîtes, le code lit leur couleur.

`PerceptionCouleur` expose la même méthode find() que le PerceptionService
(le service de perception d'AXIS-OS) : la façade de l'app de tri
(facade.py) l'utilise sans rien savoir de ce découpage. La requête qu'elle
reçoit est le nom de la couleur, d'où ConfigFacade(requete="{couleur}").

Les valeurs dites « mesurées » ou « constatées sur GPU » viennent des essais
de la scène de démonstration en simulation physique (Isaac Sim, sur carte
graphique) ; la scène est décrite dans docs/demo_tri_usine.md. Tests :
test/test_couleur_colis.py.
"""
from __future__ import annotations

import colorsys
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from axis_os.hal.core.frames import FRAME_WORLD
from axis_os.services.base import ServiceResult

# Couleurs de tri reconnues, chacune décrite par une ou plusieurs plages de
# teinte, en degrés sur le cercle des couleurs du modèle HSV (teinte,
# saturation, valeur ; 0° = rouge, 120° = vert, 240° = bleu). Le rouge est
# à cheval sur 0°, d'où ses deux plages. Les seuils de saturation et de
# luminosité, communs à toutes les couleurs, suivent.
TEINTES: Dict[str, Tuple[Tuple[float, float], ...]] = {
    "rouge": ((0.0, 18.0), (340.0, 360.0)),
    "vert": ((85.0, 160.0),),
    "bleu": ((195.0, 255.0),),
    "jaune": ((45.0, 70.0),),
}
# Saturation minimale (0 = gris, 1 = couleur pure) pour qu'un pixel compte
# comme coloré. Pourquoi 0.25 : mesuré sur GPU, sous un éclairage fort, un
# carton rouge devient rose (saturation 0.38) et un carton bleu devient bleu
# ciel (0.40), alors que le gris et le blanc de la scène restent sous 0.10.
SATURATION_MIN = 0.25        # en dessous : gris, blanc
# Luminosité minimale (0 = noir). Volontairement basse : un carton dans
# l'ombre du robot garde sa teinte mais devient sombre (constaté sur GPU).
# Le noir pur reste exclu, cette fois par le seuil de saturation.
LUMINOSITE_MIN = 0.08


def couleur_de(rgb: Tuple[float, float, float],
               couleurs: Iterable[str] = TEINTES) -> Optional[str]:
    """Couleur de tri d'un pixel, ou None si sa couleur n'est pas franche.

    rgb : composantes rouge, vert, bleu, chacune dans [0, 1]. Seules les
    couleurs de `couleurs` sont envisagées. Un pixel trop gris, trop sombre
    ou hors de toutes les plages de teinte rend None.
    """
    h, s, v = colorsys.rgb_to_hsv(*rgb)
    if s < SATURATION_MIN or v < LUMINOSITE_MIN:
        return None
    deg = h * 360.0
    for nom in couleurs:
        if any(a <= deg < b for a, b in TEINTES.get(nom, ())):
            return nom
    return None


def couleur_boite(data: bytes, largeur: int, hauteur: int,
                  bbox: Sequence[int], couleurs: Iterable[str] = TEINTES,
                  part_centrale: float = 0.85, part_coloree_min: float = 0.08,
                  majorite: float = 0.7) -> Optional[str]:
    """Couleur d'un objet, lue dans sa boîte englobante, ou None.

    data : image au format rgb8 (trois octets par pixel, rouge, vert, bleu,
    ligne après ligne) de `largeur` × `hauteur` pixels. bbox : boîte
    (x, y, largeur, hauteur), en pixels.

    Seuls les pixels franchement colorés votent. Pourquoi : la première
    version prenait la couleur majoritaire au centre de la boîte, tous
    pixels confondus. Sur GPU, la face du carton tournée vers le robot était
    dans l'ombre, donc noire ; seul le rebord éclairé portait la couleur, et
    chaque carton était rejeté. Les pixels sombres, gris ou beiges (ombre,
    table, fond) ne votent donc pas.

    Garde-fous, pour ne jamais envoyer un colis au mauvais poste :
      - au moins `part_coloree_min` des pixels examinés doivent être
        colorés, sinon None (un objet gris n'a pas de couleur de tri) ;
      - la couleur gagnante doit réunir au moins `majorite` des pixels
        colorés, sinon None (une boîte à cheval sur deux cartons est
        ambiguë).
    Les bords de la boîte sont exclus, seule sa `part_centrale` est
    examinée : une détection déborde souvent sur l'objet voisin.
    """
    x, y, w, h = (int(v) for v in bbox)
    mx, my = int(w * (1 - part_centrale) / 2), int(h * (1 - part_centrale) / 2)
    x0, x1 = max(0, x + mx), min(largeur, x + w - mx)
    y0, y1 = max(0, y + my), min(hauteur, y + h - my)
    if x1 <= x0 or y1 <= y0:
        return None
    votes: Dict[str, int] = {}
    # Un pixel sur `pas` dans chaque direction, soit environ 25 points le
    # long du plus petit côté de la zone examinée : assez pour voter sans
    # parcourir toute la boîte.
    pas = max(1, min(x1 - x0, y1 - y0) // 25)      # ~600 pixels examinés
    n = 0
    for yy in range(y0, y1, pas):
        for xx in range(x0, x1, pas):
            i = 3 * (yy * largeur + xx)
            c = couleur_de((data[i] / 255, data[i + 1] / 255, data[i + 2] / 255), couleurs)
            n += 1
            if c is not None:
                votes[c] = votes.get(c, 0) + 1
    colores = sum(votes.values())
    if not n or colores < part_coloree_min * n:
        return None
    meilleure = max(votes, key=lambda c: votes[c])
    if votes[meilleure] < majorite * colores:
        return None
    return meilleure


def mesure_couleur(data: bytes, largeur: int, hauteur: int, bbox: Sequence[int]) -> str:
    """Texte de diagnostic, qui dit pourquoi une couleur est rejetée.

    Donne le RGB moyen au centre de la boîte (la moitié centrale, en largeur
    comme en hauteur) et, pour chaque couleur, la part des pixels jugés de
    cette couleur (« aucune » pour ceux sans couleur franche). Ce texte sert
    seulement à comprendre ; il n'intervient pas dans la décision.
    """
    x, y, w, h = (int(v) for v in bbox)
    x0, x1 = max(0, x + w // 4), min(largeur, x + 3 * w // 4)
    y0, y1 = max(0, y + h // 4), min(hauteur, y + 3 * h // 4)
    if x1 <= x0 or y1 <= y0:
        return "hors image"
    somme, n, votes = [0.0, 0.0, 0.0], 0, {}
    pas = max(1, min(x1 - x0, y1 - y0) // 20)
    for yy in range(y0, y1, pas):
        for xx in range(x0, x1, pas):
            i = 3 * (yy * largeur + xx)
            rgb = (data[i] / 255, data[i + 1] / 255, data[i + 2] / 255)
            for k in range(3):
                somme[k] += rgb[k]
            c = couleur_de(rgb)
            votes[c] = votes.get(c, 0) + 1
            n += 1
    moy = tuple(round(v / n, 2) for v in somme)
    parts = ", ".join(f"{k or 'aucune'} {100 * v // n}%" for k, v in
                      sorted(votes.items(), key=lambda kv: -kv[1]))
    return f"RGB moyen {moy} — {parts}"


def _recouvrement(a, b) -> float:
    """Recouvrement de deux boîtes (x, y, largeur, hauteur) : aire de leur
    intersection divisée par celle de leur union, 0 si elles sont
    disjointes."""
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ix = max(0, min(ax + aw, bx + bw) - max(ax, bx))
    iy = max(0, min(ay + ah, by + bh) - max(ay, by))
    inter = ix * iy
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


def sans_doublons(dets: List, seuil: float = 0.5) -> List:
    """Retire les détections en double, en gardant la plus sûre.

    Plusieurs formulations de la requête (« a box », « a cardboard box »)
    voient souvent la même boîte. Les détections sont parcourues de la plus
    confiante à la moins confiante ; une détection qui recouvre une
    détection déjà gardée à `seuil` ou plus (voir _recouvrement) est
    écartée.
    """
    gardees: List = []
    for d in sorted(dets, key=lambda d: -d.confidence):
        if all(_recouvrement(d.bbox, g.bbox) < seuil for g in gardees):
            gardees.append(d)
    return gardees


class PerceptionCouleur:
    """find(couleur) : boîtes détectées par l'IA, filtrées par leur couleur.

    Une recherche interroge le détecteur une fois par formulation de
    `requetes`, retire les doublons, puis classe chaque détection d'après la
    couleur lue dans l'image de la caméra. Le résultat est gardé en mémoire
    pour l'instant `now_ns` : chercher plusieurs couleurs au même instant ne
    relance pas l'inférence (le calcul du réseau de neurones), coûteuse.
    """

    def __init__(self, perception, camera, couleurs: Iterable[str],
                 requetes: Sequence[str] = ("a box", "a cardboard box"),
                 surface_max: float = 0.25):
        """perception : détecteur d'objets, avec la méthode find() du
        PerceptionService ; camera : source des images rgb8 dont on lit les
        pixels ; couleurs : couleurs de tri reconnues (clés de TEINTES) ;
        requetes : formulations envoyées au détecteur.

        surface_max : fraction de l'image au-delà de laquelle une détection
        ne peut pas être un carton. Constaté sur GPU : OWL-ViT rend aussi une
        boîte couvrant toute l'image ; le sol de la scène étant bleu, elle
        aurait pu passer pour un carton bleu fantôme."""
        self.surface_max = surface_max
        self.perception, self.camera = perception, camera
        self.couleurs = list(couleurs)
        self.requetes = tuple(requetes)
        self._cache_ns: Optional[int] = None
        self._cache: List = []
        self.stats = {"inferences": 0, "sans_couleur": 0}
        # Ce qu'a vu la dernière recherche, détection par détection. Il sert
        # à comprendre un « rien trouvé » sans deviner : l'IA n'a-t-elle rien
        # vu, la couleur a-t-elle été rejetée, la position filtrée ?
        self.diagnostic: dict = {}

    def _detections(self, now_ns, frame) -> Tuple[List, Optional[str]]:
        """Détections de l'instant `now_ns`, classées par couleur.

        Rend la liste des couples (couleur ou None, détection) et, si le
        détecteur n'a rien rendu du tout, la raison de son dernier échec.
        Remplit aussi self.diagnostic."""
        if now_ns is not None and now_ns == self._cache_ns:
            return self._cache, None
        dets, raison = [], None
        diag = {"par_requete": {}, "raisons": [], "detections": []}
        for rq in self.requetes:
            r = self.perception.find(rq, now_ns=now_ns, frame=frame)
            self.stats["inferences"] += 1
            if r.ok:
                dets += list(r.value or [])
                diag["par_requete"][rq] = len(r.value or [])
            else:
                raison = r.reason
                diag["par_requete"][rq] = 0
            if r.reason:
                diag["raisons"].append(f"{rq} : {r.reason}")
        dets = sans_doublons(dets)
        img = self.camera.capture()
        classees = []
        if img is not None:
            aire_img = float(img.width * img.height)
            for d in dets:
                if d.bbox[2] * d.bbox[3] > self.surface_max * aire_img:
                    diag["detections"].append({"couleur": "écartée : trop grande",
                                               "confiance": round(float(d.confidence), 3),
                                               "bbox": tuple(int(v) for v in d.bbox),
                                               "position": None})
                    continue
                c = couleur_boite(img.data, img.width, img.height, d.bbox, self.couleurs)
                mesure = mesure_couleur(img.data, img.width, img.height, d.bbox)
                if c is None:
                    self.stats["sans_couleur"] += 1
                classees.append((c, d))
                p = d.position_3d
                diag["detections"].append({
                    "couleur": c, "confiance": round(float(d.confidence), 3),
                    "bbox": tuple(int(v) for v in d.bbox),
                    "position": None if p is None else
                    (round(p.x, 2), round(p.y, 2), round(p.z, 2)),
                    "mesure": mesure})
        diag["image"] = img is not None
        self.diagnostic = diag
        self._cache_ns, self._cache = now_ns, classees
        return classees, (raison if not dets else None)

    def find(self, query: str, now_ns: Optional[int] = None,
             frame: str = FRAME_WORLD) -> ServiceResult:
        """Détections de la couleur `query`, de la plus confiante à la
        moins confiante. Échoue seulement si le détecteur a échoué et
        qu'aucune détection n'a pu être classée.

        frame : repère des positions rendues ; FRAME_WORLD est le repère
        « monde », fixe, celui de la carte."""
        classees, raison = self._detections(now_ns, frame)
        if raison and not classees:
            return ServiceResult(ok=False, value=[], reason=raison)
        gardees = [d for c, d in classees if c == query]
        return ServiceResult(ok=True, value=sorted(gardees, key=lambda d: -d.confidence))
