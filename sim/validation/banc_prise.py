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
        self.en_cours = False

    def debut(self) -> None:
        self.reel = {"left": None, "right": None}
        self.estimes = {"left": None, "right": None}
        self.lacets = {}
        self.en_cours = True

    def noter_lacet(self, phase: str, lacet_deg: Optional[float]) -> None:
        """Lacet réel du carton (degrés) à l'entrée d'une phase, une fois."""
        if self.en_cours and lacet_deg is not None and phase not in self.lacets:
            self.lacets[phase] = float(lacet_deg)

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
        for h in entetes:
            a = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
            b = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
            for cote, corps in self.corps.items():
                if (a.startswith(corps) and any(b.startswith(c) for c in self._cartons)) or \
                        (b.startswith(corps) and any(a.startswith(c) for c in self._cartons)):
                    for i in range(h.contact_data_offset,
                                   h.contact_data_offset + h.num_contact_data):
                        imp = donnees[i].impulse
                        force[cote] += math.sqrt(imp[0] ** 2 + imp[1] ** 2 + imp[2] ** 2) / self.dt
        self._force = force

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
