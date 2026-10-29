"""
Banc Isaac Sim T1, côté prise : gains de prise des poignets et le juge des
contacts. Rien ici n'est lu par l'OS (axis_os/) ; un test le vérifie.

Gains de prise : pendant FERMETURE et jusqu'au dépôt, le ManipulationService
demande des poignets (et un coude) plus raides (MesuresBras.raideur). La
commande bas niveau du vrai G1 le permet (kp et kd dans chaque LowCmd) ; le
banc le fait par les gains PD d'Isaac.

Juge : capteur de contact PhysX sur chaque main, VÉRITÉ DE LA SIMULATION,
pour noter le contact estimé par l'OS (manip.contacts_estimes) : à chaque
prise, par main, l'instant et le signal de l'estimation, l'instant et la
force du contact réel ; le bilan compte vrais contacts, contacts manqués et
faux contacts.
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
# cédaient sur GPU (2026-10-29), et le coude.
RAIDIS = ("wrist_roll_joint", "wrist_pitch_joint", "wrist_yaw_joint", "elbow_joint")


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
    """Contacts réels des mains (capteur du banc) contre contacts estimés
    par l'OS, prise par prise.

    lire(cote) -> force (N) du contact entre la main et un carton, 0 sans
    contact, None si le capteur ne répond pas.
    Une prise va de l'entrée en FERMETURE à sa sortie. Par main :
      vrai contact   : estimé, et réel au plus tard `tolerance_s` après ;
      faux contact   : estimé sans contact réel à ce moment-là ;
      contact manqué : réel, jamais estimé pendant la fermeture.
    """

    def __init__(self, lire: Callable[[str], Optional[float]], seuil_n: float = 2.0,
                 tolerance_s: float = 0.1):
        self._lire = lire
        self.seuil_n = seuil_n
        self.tolerance_s = tolerance_s
        self.compteurs = {"contacts_vrais": 0, "contacts_manques": 0, "faux_contacts": 0}
        self.disponible = True
        self.reel: Dict[str, Optional[Dict[str, float]]] = {}
        self.en_cours = False

    def debut(self) -> None:
        self.reel = {"left": None, "right": None}
        self.en_cours = True

    def pas(self, t: float) -> None:
        if not self.en_cours:
            return
        for cote in ("left", "right"):
            f = self._lire(cote)
            if f is None:
                self.disponible = False
                continue
            if self.reel[cote] is None and f >= self.seuil_n:
                self.reel[cote] = {"t": t, "force": float(f)}

    def fin(self, estimes: Dict[str, Optional[Dict]]) -> List[str]:
        """Clôt la prise ; une ligne de journal par main."""
        self.en_cours = False
        lignes = []
        for cote, nom in (("left", "G"), ("right", "D")):
            e, r = estimes.get(cote), self.reel.get(cote)
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
        return lignes


def capteurs_contact_isaac(stage, mains: Dict[str, str], cartons: List[str],
                           dt: float) -> Optional[Callable[[str], Optional[float]]]:
    """Capteurs de contact PhysX sur les corps des mains (`mains` : côté ->
    chemin du prim), filtrés sur les `cartons` (chemins des prims). Rend
    lire(cote) pour JugeContacts, ou None si Isaac n'a pas de capteur de
    contact (le juge est alors absent, la prise n'en dépend pas)."""
    try:
        from isaacsim.sensors.physics import ContactSensor
    except ImportError:
        try:
            from omni.isaac.sensor import ContactSensor
        except ImportError:
            return None
    capteurs = {}
    for cote, chemin in mains.items():
        try:
            c = ContactSensor(prim_path=f"{chemin}/juge_contact", name=f"juge_{cote}",
                              min_threshold=0.0, max_threshold=1e7, radius=-1)
            c.initialize()
            c.add_raw_contact_data_to_frame()
            capteurs[cote] = c
        except Exception as exc:            # noqa: BLE001 — banc sans juge
            print(f"  [juge] capteur de contact {cote} indisponible : {exc}", flush=True)
            return None

    def lire(cote: str) -> Optional[float]:
        try:
            trame = capteurs[cote].get_current_frame()
        except Exception:                   # noqa: BLE001
            return None
        brut = trame.get("contacts")
        if brut is None:
            return float(trame.get("force", 0.0)) if trame.get("in_contact") else 0.0
        f = 0.0
        for k in brut:
            corps = f"{k.get('body0', '')} {k.get('body1', '')}"
            if any(c in corps for c in cartons):
                imp = k.get("impulse", (0.0, 0.0, 0.0))
                f += math.sqrt(sum(float(v) ** 2 for v in imp)) / dt
        return f
    return lire
