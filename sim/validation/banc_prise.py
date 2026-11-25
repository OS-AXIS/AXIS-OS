"""
Banc Isaac Sim T1, côté prise : gains de prise des poignets et le juge des
contacts. Rien ici n'est lu par l'OS (axis_os/) ; un test le vérifie.

Gains de prise : pendant FERMETURE et jusqu'au dépôt, le ManipulationService
demande des poignets (et un coude) plus raides (MesuresBras.raideur). La
commande bas niveau du vrai G1 le permet (kp et kd dans chaque LowCmd) ; le
banc le fait par les gains PD d'Isaac.

Juge : contacts PhysX de chaque main avec les cartons, VÉRITÉ DE LA
SIMULATION, pour noter le contact estimé par l'OS (manip.contacts_estimes) :
à chaque prise, par main, l'instant et le signal de l'estimation, l'instant
et la force du contact réel, et le lacet réel du carton à chaque phase
(placement, contact, serrage, fin : tourne-t-il ?) ; le bilan compte vrais
contacts, contacts manqués et faux contacts.

Constat GPU (Isaac Sim 6.1, PR 16) : « juge des contacts : indisponible »,
sans la raison. Causes possibles, toutes traitées ici et dites au journal :
l'extension des capteurs n'est pas chargée en mode sans écran (import
impossible), les capteurs étaient créés après world.reset() (le rapport de
contact PhysX doit être posé avant le départ de la physique), la main
(rubber_hand) fusionnée dans le poignet par l'import de l'URDF.
"""
from __future__ import annotations

import math
import os
from typing import Callable, Dict, List, Optional, Tuple

# Plus grand gain kp du pilote du vrai G1 (_DEFAULT_KP de g1_dds_actuator :
# les genoux) : les gains de prise ne le dépassent pas. Le couple reste borné
# par la limite d'effort de chaque moteur (5 N·m pour le tangage et le lacet
# des poignets).
KP_MAX_PILOTE = 100.0
# Articulations raidies pendant la prise, par côté : les poignets, qui
# cédaient sur GPU (2026-10-29), le coude, et le lacet d'épaule (la plus
# souple du bras à la main, aux gains du banc : 40 N·m/rad pour un bras de
# levier de ~0,22 m ; le serrage passe par lui autant que par le roulis).
RAIDIS = ("wrist_roll_joint", "wrist_pitch_joint", "wrist_yaw_joint", "elbow_joint",
          "shoulder_yaw_joint")


def mains_caoutchouc(stage, racine: str = "/World/g1") -> int:
    """Lie aux mains du robot le matériau des mains caoutchouc du G1 (W3 :
    frottement statique 1,2, dynamique 1,0). AVANT world.reset() : ignoré
    après. Rend le nombre de prims liés (bancs T1 et de posture)."""
    from isaacsim.core.api.materials import PhysicsMaterial
    from pxr import UsdShade
    PhysicsMaterial(prim_path="/World/Materiaux/caoutchouc",
                    static_friction=1.2, dynamic_friction=1.0)
    mat = UsdShade.Material(stage.GetPrimAtPath("/World/Materiaux/caoutchouc"))
    n = 0
    for prim in stage.Traverse():
        nom = prim.GetName()
        if racine in str(prim.GetPath()) and ("rubber_hand" in nom or "wrist_yaw_link" in nom):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                mat, UsdShade.Tokens.weakerThanDescendants, "physics")
            n += 1
    return n


def profil_robot_banc(graine: int) -> Tuple[str, str]:
    """Profil du robot simulé des bancs (axis_os/services/profil_robot.py) :
    (chemin, identifiant). Ce qui distingue un robot du banc d'un autre, ce
    sont les erreurs de ses couples estimés, tirées de la graine
    (bruit_couples.BruitCouples) : une graine, un robot, un profil, gardé
    hors du dépôt d'un lancement à l'autre (sa calibration de charge se fait
    une fois, comme sur le vrai robot)."""
    robot_id = f"isaac_g1_graine{graine}"
    return (os.path.join(os.path.expanduser("~"), ".axis_os", "robots", f"{robot_id}.json"),
            robot_id)


def poser_devant(robot, carton, plateau, distance_m: float, plateau_m: float,
                 hauteur_m: float):
    """Le carton sur son plateau (cinématique), devant le robot, là où T1
    prend un carton (bord du plateau contre sa face avant) ; rend ce que la
    perception en dirait : (centre, lacet), repère monde. Carton et plateau
    créés avant le départ de la physique, loin, à l'échelle (profondeur,
    largeur, hauteur) dans le repère du robot."""
    import numpy as np
    pr, qr = robot.get_world_pose()
    lacet_r = 2.0 * math.atan2(float(qr[3]), float(qr[0]))
    avant = np.array([math.cos(lacet_r), math.sin(lacet_r), 0.0])
    q_lacet = np.array([math.cos(lacet_r / 2), 0.0, 0.0, math.sin(lacet_r / 2)])
    sol = np.array([float(pr[0]), float(pr[1]), 0.0])
    centre = sol + distance_m * avant + np.array([0.0, 0.0, plateau_m + hauteur_m / 2])
    plateau.set_world_pose(position=sol + (distance_m + 0.10) * avant
                           + np.array([0.0, 0.0, plateau_m / 2]), orientation=q_lacet)
    carton.set_world_pose(position=centre, orientation=q_lacet)
    carton.set_linear_velocity(np.zeros(3))
    carton.set_angular_velocity(np.zeros(3))
    return tuple(float(v) for v in centre), lacet_r


def preparer_scene(world, stage, avant_reset: Optional[Callable[[], None]] = None,
                   journal: Callable[[str], None] = print) -> None:
    """Préparation commune des bancs (T1 et posture), une fois TOUS les
    corps rigides posés (world.scene.add), dans l'ordre qui tient sur GPU :
    mains caoutchouc et éclairage, puis ce qui doit précéder le départ de la
    physique (`avant_reset` : capteurs de contact du juge), puis
    world.reset(). Trois défauts de banc en trois soirs venaient de cet
    ordre, recopié à la main dans chaque banc (essai GPU du 20 novembre
    2026 : mains posées avant les corps rigides, « Simulation view object is
    invalidated »)."""
    n = mains_caoutchouc(stage)
    journal(f"mains caoutchouc : {n} prim(s) lié(s)")
    for ligne in eclairer(stage):
        journal(ligne)
    if avant_reset is not None:
        avant_reset()
    world.reset()


def demarrer_services(services, journal: Callable[[str], None] = print) -> None:
    """Démarre les services de l'OS du banc (locomotion, manipulation,
    perception…), dans l'ordre donné, et le dit au journal : un banc qui
    oubliait d'en démarrer un ne le disait nulle part (essai GPU du
    20 novembre 2026)."""
    for nom, service in services:
        service.start()
        journal(f"service démarré : {nom}")


def objets_inconnus(objets, obstacles) -> List[str]:
    """Les objets posés devant le robot (nom, (x, y) repère monde) que la
    carte du site n'explique pas (sur aucun de ses `obstacles`, rectangles
    de la carte) : vide si tous reposent sur une table connue. Le gardien
    de proximité arrêterait le robot devant un objet inconnu (essai GPU du
    20 novembre 2026, 20 h 27 : plateau de la calibration de charge)."""
    return [nom for nom, (x, y) in objets
            if not any(o.xmin <= x <= o.xmax and o.ymin <= y <= o.ymax for o in obstacles)]


def chevauchements(position, objets, taille) -> List[str]:
    """Les objets déjà posés (nom, (x, y) repère monde) dont l'empreinte
    chevaucherait celle d'un objet posé en `position` (même `taille` :
    largeur, profondeur, alignées sur les axes). Deux corps posés l'un dans
    l'autre, la physique en éjecte un (essai GPU du 25 novembre 2026,
    16 h 43 : la masse de référence posée sur l'emplacement d'un carton,
    éjectée au sol dans la zone de marche, vue par le gardien comme un
    obstacle inconnu ; robot arrêté 90 s)."""
    x, y = position
    largeur, profondeur = taille
    return [nom for nom, (xo, yo) in objets
            if abs(xo - x) < largeur and abs(yo - y) < profondeur]


def plateau_site(pose_robot, distance_m: float, plateau_m: float, profondeur_m: float = 0.40,
                 largeur_m: float = 0.60, chargement_m: float = 0.25):
    """Le plateau que poser_devant pose devant le robot, déclaré comme les
    tables du site : son rectangle (repère monde, aligné sur les axes, qui
    le contient) et sa hauteur de chargement. `pose_robot` : (x, y, lacet)."""
    from axis_os.services.navigation.service import Rectangle
    x, y, lacet = pose_robot
    cx, cy = x + (distance_m + 0.10) * math.cos(lacet), y + (distance_m + 0.10) * math.sin(lacet)
    c, s_ = abs(math.cos(lacet)), abs(math.sin(lacet))
    dx = (profondeur_m * c + largeur_m * s_) / 2.0
    dy = (profondeur_m * s_ + largeur_m * c) / 2.0
    return Rectangle(cx - dx, cy - dy, cx + dx, cy + dy, zmax=plateau_m + chargement_m)


def eclairer(stage) -> List[str]:
    """L'éclairage des bancs (scène T1, puis banc de posture) : une lumière
    ambiante (dôme) et quatre lumières en diagonale, une par coin, plongée de
    50°, comme les rampes d'un entrepôt. Rend le sens réel de chaque lumière,
    pour le journal.

    Historique GPU (scène T1) : la seule lumière venait de derrière la table,
    la face des cartons tournée vers le robot était noire sur la caméra de
    tête (RGB 0,0,0), avec une lumière de face à 60° puis presque verticale
    (15°), puis deux en diagonale du côté du robot (±40°) — les faces
    latérales s'allumaient, les faces avant restaient noires ; le dôme seul
    ne suffit pas. Avec une lumière par coin, toute face verticale en reçoit
    au moins deux, quel que soit le sens de rotation retenu par USD ; l'ombre
    du robot n'en masque qu'une à la fois. Banc de posture (essai du
    19 novembre 2026, 21 h 51) : avec le seul dôme du pont, la caméra
    spectateur ne rendait rien après 120 images (T1 : dès la 2e)."""
    from pxr import Gf, UsdGeom, UsdLux
    dome = UsdLux.DomeLight.Define(stage, "/World/eclairage_ambiant")
    dome.CreateIntensityAttr(400.0)   # 900 délavait les couleurs
    sens = []
    for nom_l, lacet in (("eclairage_1", 45.0), ("eclairage_2", -45.0),
                         ("eclairage_3", 135.0), ("eclairage_4", -135.0)):
        lum = UsdLux.DistantLight.Define(stage, f"/World/{nom_l}")
        lum.CreateIntensityAttr(700.0)
        lum.CreateAngleAttr(1.0)
        xf_l = UsdGeom.Xformable(lum)
        xf_l.AddRotateZOp().Set(lacet)
        xf_l.AddRotateXOp().Set(50.0)
        d_l = xf_l.ComputeLocalToWorldTransform(0.0).TransformDir(Gf.Vec3d(0.0, 0.0, -1.0))
        sens.append(f"éclairage : {nom_l} vers ({d_l[0]:+.2f}, {d_l[1]:+.2f}, {d_l[2]:+.2f})")
    return sens


def gains_prise(normaux: Dict[str, Tuple[float, float]], facteur: float = 2.0
                ) -> Dict[str, Tuple[float, float]]:
    """Gains (kp, kd) de prise : kp × `facteur`, borné à KP_MAX_PILOTE, kd
    × √(kp de prise / kp) (même amortissement relatif). Les autres articulations
    gardent leurs gains."""
    sortie = dict(normaux)
    for j, (kp, kd) in normaux.items():
        if j.endswith(RAIDIS) and j.startswith(("left_", "right_")):
            kp2 = min(kp * facteur, max(kp, KP_MAX_PILOTE))
            sortie[j] = (kp2, kd * math.sqrt(kp2 / kp) if kp > 0 else kd)
    return sortie


class JugeContacts:
    """Contacts réels des mains (capteurs du banc) contre contacts estimés
    par l'OS, prise par prise.

    lire(cote) -> force (N) du contact entre la main et un carton, 0 sans
    contact, None si le capteur ne répond pas.
    Une prise va de l'entrée en PLACEMENT à sa fin (tenue, échec, lâchée) ;
    les contacts réels sont relevés pendant la FERMETURE, là où l'OS les
    estime. Par main :
      vrai contact   : estimé, et réel au plus tard `tolerance_s` après ;
      faux contact   : estimé sans contact réel à ce moment-là ;
      contact manqué : réel, jamais estimé pendant la fermeture.
    Les contacts estimés sont gardés dès qu'ils paraissent (pas(estimes=…)) :
    l'app réinitialise la prise dans le pas même de son échec, et ses
    estimations avec. Constaté sur GPU (essai T1 du 5 novembre 2026) : 10
    « contacts manqués » sur 10 mains, toutes estimées en fait (recalage au
    toucher et serrage à chaque essai), effacées avant que le juge les lise.
    """

    def __init__(self, lire: Callable[[str], Optional[float]], seuil_n: float = 2.0,
                 tolerance_s: float = 0.1):
        self._lire = lire
        self.seuil_n = seuil_n
        self.tolerance_s = tolerance_s
        self.compteurs = {"contacts_vrais": 0, "contacts_manques": 0, "faux_contacts": 0}
        self.disponible = True
        self.reel: Dict[str, Optional[Dict[str, float]]] = {"left": None, "right": None}
        self.estimes: Dict[str, Optional[Dict]] = {"left": None, "right": None}
        self.lacets: Dict[str, float] = {}
        # Orientation réelle du carton dans le repère du bassin, par phase :
        # (roulis, tangage) les plus grands en valeur absolue (degrés). Le
        # tangage, c'est la rotation autour de la ligne des mains (essai GPU
        # du 5 novembre 2026, 16 h 06 : levé, le carton y a pivoté, puis est
        # tombé ; le lacet seul ne le montrait pas).
        self.orientations: Dict[str, Tuple[float, float]] = {}
        self.en_cours = False

    def debut(self) -> None:
        self.reel = {"left": None, "right": None}
        self.estimes = {"left": None, "right": None}
        self.lacets = {}
        self.orientations = {}
        self.en_cours = True

    def noter_lacet(self, phase: str, lacet_deg: Optional[float]) -> None:
        """Lacet réel du carton (degrés) à l'entrée d'une phase, une fois."""
        if self.en_cours and lacet_deg is not None and phase not in self.lacets:
            self.lacets[phase] = float(lacet_deg)

    def noter_orientation(self, phase: str, roulis_deg: Optional[float],
                          tangage_deg: Optional[float]) -> None:
        """Orientation réelle du carton pendant une phase : garde les plus
        grands roulis et tangage (valeur absolue, signe conservé)."""
        if not self.en_cours or roulis_deg is None or tangage_deg is None:
            return
        r0, t0 = self.orientations.get(phase, (0.0, 0.0))
        self.orientations[phase] = (roulis_deg if abs(roulis_deg) > abs(r0) else r0,
                                    tangage_deg if abs(tangage_deg) > abs(t0) else t0)

    def pas(self, t: float, fermeture: bool = True,
            estimes: Optional[Dict[str, Optional[Dict]]] = None) -> None:
        """Un pas : `estimes`, les contacts estimés par l'OS à cet instant
        (gardés dès qu'ils paraissent) ; contacts réels relevés pendant la
        fermeture."""
        if not self.en_cours:
            return
        for cote, e in (estimes or {}).items():
            if e is not None and self.estimes.get(cote) is None:
                self.estimes[cote] = dict(e)
        if not fermeture:
            return
        for cote in ("left", "right"):
            f = self._lire(cote)
            if f is None:
                self.disponible = False
                continue
            if self.reel[cote] is None and f >= self.seuil_n:
                self.reel[cote] = {"t": t, "force": float(f)}

    def fin(self, estimes: Dict[str, Optional[Dict]]) -> List[str]:
        """Clôt la prise ; une ligne de journal par main, puis les lacets."""
        self.en_cours = False
        lignes = []
        for cote, nom in (("left", "G"), ("right", "D")):
            e, r = self.estimes.get(cote) or estimes.get(cote), self.reel.get(cote)
            if e is not None and r is not None and r["t"] <= e["t"] + self.tolerance_s:
                verdict = "vrai contact"
                self.compteurs["contacts_vrais"] += 1
            elif e is not None:
                verdict = "FAUX CONTACT"
                self.compteurs["faux_contacts"] += 1
            elif r is not None:
                verdict = "CONTACT MANQUÉ"
                self.compteurs["contacts_manques"] += 1
            else:
                verdict = "aucun contact"
            lignes.append(
                f"juge (vérité simulation), main {nom} : contact estimé par l'OS "
                + (f"à {e['t']:.2f} s ({e['signal']})" if e else "non")
                + ", contact réel "
                + (f"à {r['t']:.2f} s ({r['force']:.1f} N)" if r else
                   "non" if self.disponible else "inconnu (capteur indisponible)")
                + f" — {verdict}")
        if self.lacets:
            lignes.append("juge (vérité simulation), lacet réel du carton : " + ", ".join(
                f"{p} {v:+.1f}°" for p, v in self.lacets.items()))
        if self.orientations:
            lignes.append("juge (vérité simulation), carton au plus penché (tangage autour de la "
                          "ligne des mains / roulis, repère du bassin) : " + ", ".join(
                              f"{p} {t:+.1f}° / {r:+.1f}°"
                              for p, (r, t) in self.orientations.items()))
        return lignes


class CapteursContactIsaac:
    """Force de contact de chaque main avec les cartons, dans Isaac Sim.

    preparer() AVANT world.reset() : trouve les corps des mains sous le
    prim du robot, pose le rapport de contact PhysX (PhysxContactReportAPI,
    seuil 0) et s'abonne aux contacts ; à défaut, le capteur de contact
    d'Isaac (extension activée d'abord). lire(cote) rend la force (N) du
    dernier pas de physique, None si aucun moyen n'a marché ; `raisons` dit
    pourquoi, pour le journal."""

    def __init__(self, dt: float):
        self.dt = dt
        self.corps: Dict[str, str] = {}
        self.methode: Optional[str] = None
        self.raisons: List[str] = []
        self._cartons: List[str] = []
        self._force = {"left": 0.0, "right": 0.0}
        # Point de contact réel de chaque main (repère monde) : barycentre des
        # points du rapport PhysX, pondérés par leur impulsion ; None sans
        # contact. Où la main touche-t-elle le carton (ReglagesPrise.contact_main_m) ?
        self._point: Dict[str, Optional[Tuple[float, float, float]]] = {"left": None,
                                                                        "right": None}
        self._capteurs: Dict[str, object] = {}
        self._abonnement = None
        self._physx = None

    def preparer(self, stage, racine: str, cartons: List[str]) -> None:
        from pxr import UsdPhysics
        self._cartons = list(cartons)
        for cote in ("left", "right"):
            vus = []
            for nom in (f"{cote}_rubber_hand", f"{cote}_wrist_yaw_link"):
                for prim in stage.Traverse():
                    if prim.GetName() != nom or not str(prim.GetPath()).startswith(racine):
                        continue
                    vus.append(str(prim.GetPath()))
                    p = prim
                    while p and p.IsValid() and not p.HasAPI(UsdPhysics.RigidBodyAPI):
                        p = p.GetParent()          # main fusionnée : son corps rigide
                    if p and p.IsValid() and str(p.GetPath()).startswith(racine):
                        self.corps[cote] = str(p.GetPath())
                        break
                if cote in self.corps:
                    break
            if cote not in self.corps:
                self.raisons.append(f"corps rigide de la main {cote} introuvable sous {racine} "
                                    f"(prims vus : {vus or 'aucun'})")
        if len(self.corps) < 2:
            return
        try:
            from omni.physx import get_physx_simulation_interface
            from pxr import PhysxSchema
            for chemin in self.corps.values():
                api = PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(chemin))
                api.CreateThresholdAttr().Set(0.0)
            self._physx = get_physx_simulation_interface()
            if not hasattr(self._physx, "get_contact_report"):
                # Sans relevé à la demande : abonnement (un pas sans contact
                # peut alors ne rien envoyer ; la force reste celle d'avant).
                self._abonnement = self._physx.subscribe_contact_report_events(
                    self._sur_contacts)
            self.methode = "rapport de contact PhysX"
            return
        except Exception as exc:                 # noqa: BLE001 — on essaie l'autre moyen
            self.raisons.append(f"rapport de contact PhysX : {type(exc).__name__}: {exc}")
        try:
            try:
                from isaacsim.core.utils.extensions import enable_extension
                enable_extension("isaacsim.sensors.physics")
            except Exception as exc:             # noqa: BLE001
                self.raisons.append(f"extension isaacsim.sensors.physics : {exc}")
            from isaacsim.sensors.physics import ContactSensor
            for cote, chemin in self.corps.items():
                self._capteurs[cote] = ContactSensor(
                    prim_path=f"{chemin}/juge_contact", name=f"juge_{cote}",
                    min_threshold=0.0, max_threshold=1e7, radius=-1)
            self.methode = "capteur de contact Isaac"
        except Exception as exc:                 # noqa: BLE001
            self.raisons.append(f"capteur de contact Isaac : {type(exc).__name__}: {exc}")

    def initialiser(self) -> None:
        """APRÈS world.reset()."""
        for cote, c in list(self._capteurs.items()):
            try:
                c.initialize()
                c.add_raw_contact_data_to_frame()
            except Exception as exc:             # noqa: BLE001
                self.raisons.append(f"capteur {cote} : {type(exc).__name__}: {exc}")
                self.methode = None

    def _sur_contacts(self, entetes, donnees) -> None:
        from pxr import PhysicsSchemaTools
        force = {"left": 0.0, "right": 0.0}
        somme = {"left": [0.0, 0.0, 0.0, 0.0], "right": [0.0, 0.0, 0.0, 0.0]}
        for h in entetes:
            a = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
            b = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
            for cote, corps in self.corps.items():
                if (a.startswith(corps) and any(b.startswith(c) for c in self._cartons)) or \
                        (b.startswith(corps) and any(a.startswith(c) for c in self._cartons)):
                    for i in range(h.contact_data_offset,
                                   h.contact_data_offset + h.num_contact_data):
                        imp = donnees[i].impulse
                        f_ = math.sqrt(imp[0] ** 2 + imp[1] ** 2 + imp[2] ** 2) / self.dt
                        force[cote] += f_
                        pos = getattr(donnees[i], "position", None)
                        if pos is not None and f_ > 0.0:
                            for k in range(3):
                                somme[cote][k] += f_ * float(pos[k])
                            somme[cote][3] += f_
        self._force = force
        self._point = {c: (tuple(v / s_[3] for v in s_[:3]) if s_[3] > 0.0 else None)
                       for c, s_ in somme.items()}

    def point(self, cote: str) -> Optional[Tuple[float, float, float]]:
        """Point de contact réel de la main `cote` au dernier relevé (repère
        monde), None sans contact ou sans rapport PhysX."""
        return self._point.get(cote) if self.methode and self.methode.startswith("rapport") \
            else None

    def lire(self, cote: str) -> Optional[float]:
        """Force du dernier pas de physique ; la main gauche (lue la première
        par JugeContacts.pas) relève le rapport, la droite le réutilise."""
        if self.methode is None:
            return None
        if self.methode.startswith("rapport"):
            if cote == "left" and self._abonnement is None:
                try:
                    self._sur_contacts(*self._physx.get_contact_report())
                except Exception as exc:         # noqa: BLE001
                    self.raisons.append(f"relevé du rapport de contact : {exc}")
                    self.methode = None
                    return None
            return self._force.get(cote, 0.0)
        try:
            trame = self._capteurs[cote].get_current_frame()
        except Exception:                        # noqa: BLE001
            return None
        brut = trame.get("contacts")
        if brut is None:
            return float(trame.get("force", 0.0)) if trame.get("in_contact") else 0.0
        f = 0.0
        for k in brut:
            corps = f"{k.get('body0', '')} {k.get('body1', '')}"
            if any(c in corps for c in self._cartons):
                imp = k.get("impulse", (0.0, 0.0, 0.0))
                f += math.sqrt(sum(float(v) ** 2 for v in imp)) / self.dt
        return f
