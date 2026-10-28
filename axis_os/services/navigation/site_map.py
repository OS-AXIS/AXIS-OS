"""
AXIS-OS — Carte du sens : lieux nommés et zones du site.

La carte géométrique (Carte, service.py) dit où sont les murs ; celle-ci dit
ce que veulent dire les endroits : « quai 3 », « station de charge », la zone
où le robot n'entre jamais, celle où il marche lentement.

  Lieu    — un point (x, y) et un cap facultatif. Un lieu de type
            STATION est la station de charge (au plus une).
  Zone    — un polygone (au moins trois sommets, repère monde) :
            FORBIDDEN (le robot n'y entre jamais), SLOW (vitesse plafonnée à
            max_speed_mps), NAMED (un nom, sans règle de mouvement).

Qui écrit, qui lit : l'administrateur du site, par l'API de l'OS (publish(),
rollback()), jamais une app. Les apps lisent, par ctx.places() et ctx.zones().
Chaque publication crée une nouvelle version, vérifiée en entier avant d'être
retenue (noms uniques, nombres finis, polygones valides, aucun lieu dans une
zone interdite) ; les versions précédentes restent consultables et on peut
revenir à l'une d'elles. Avec un chemin de fichier, la carte est enregistrée
(écriture atomique) et relue au démarrage.

C'est l'OS qui fait respecter FORBIDDEN et SLOW, pour toute commande de
marche : voir zones.py (GardienZones, branché sur le service de locomotion)
et le planificateur de la navigation.
"""
from __future__ import annotations

import json
import math
import os
import tempfile
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

FORBIDDEN, SLOW, NAMED = "forbidden", "slow", "named"
ZONE_KINDS = (FORBIDDEN, SLOW, NAMED)
PLACE, STATION = "place", "charging_station"
PLACE_KINDS = (PLACE, STATION)
NAME_MAX = 64
MAX_PLACES, MAX_ZONES, MAX_VERTICES = 256, 128, 64
HISTORY_MAX = 20
COORD_MAX = 1e4


class SiteMapError(ValueError):
    """Carte refusée : la raison dit quel lieu ou quelle zone ne va pas."""


@dataclass(frozen=True)
class Place:
    name: str
    x: float
    y: float
    heading: Optional[float] = None
    kind: str = PLACE

    def to_json(self) -> dict:
        return {"name": self.name, "x": self.x, "y": self.y, "heading": self.heading,
                "kind": self.kind}


@dataclass(frozen=True)
class Zone:
    name: str
    kind: str
    polygon: Tuple[Tuple[float, float], ...]
    max_speed_mps: Optional[float] = None

    def to_json(self) -> dict:
        return {"name": self.name, "kind": self.kind,
                "polygon": [list(p) for p in self.polygon],
                "max_speed_mps": self.max_speed_mps}

    def contains(self, x: float, y: float) -> bool:
        return point_in_polygon(x, y, self.polygon)

    def distance(self, x: float, y: float) -> float:
        """Distance (m) du point au polygone : 0 dedans."""
        if self.contains(x, y):
            return 0.0
        return distance_to_polygon(x, y, self.polygon)


# ─── Géométrie ───────────────────────────────────────────────────────────

def point_in_polygon(x: float, y: float, poly: Sequence[Tuple[float, float]]) -> bool:
    """Lancer de rayon : vrai si (x, y) est dans le polygone (bords compris
    à la précision près)."""
    dedans = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xc = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if xc >= x:
                dedans = not dedans
    return dedans


def distance_to_polygon(x: float, y: float, poly: Sequence[Tuple[float, float]]) -> float:
    """Distance du point au bord le plus proche du polygone."""
    best = math.inf
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy or 1e-12
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / l2))
        best = min(best, math.hypot(ax + t * dx - x, ay + t * dy - y))
    return best


# ─── Versions ────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class SiteMapVersion:
    """Une version publiée de la carte du sens (immuable)."""
    version: int
    places: Tuple[Place, ...]
    zones: Tuple[Zone, ...]
    author: str = ""
    published_at: float = 0.0

    def place(self, name: str) -> Optional[Place]:
        return next((p for p in self.places if p.name == name), None)

    def station(self) -> Optional[Place]:
        return next((p for p in self.places if p.kind == STATION), None)

    def zones_at(self, x: float, y: float, kind: Optional[str] = None) -> List[Zone]:
        return [z for z in self.zones if (kind is None or z.kind == kind) and z.contains(x, y)]

    def forbidden(self) -> List[Zone]:
        return [z for z in self.zones if z.kind == FORBIDDEN]

    def nearest_place(self, x: float, y: float) -> Optional[Tuple[Place, float]]:
        if not self.places:
            return None
        p = min(self.places, key=lambda q: math.hypot(q.x - x, q.y - y))
        return p, math.hypot(p.x - x, p.y - y)

    def to_json(self) -> dict:
        return {"version": self.version, "author": self.author,
                "published_at": self.published_at,
                "places": [p.to_json() for p in self.places],
                "zones": [z.to_json() for z in self.zones]}


def _nom(nom) -> str:
    if not isinstance(nom, str) or not nom.strip() or len(nom) > NAME_MAX \
            or not all(c.isprintable() for c in nom):
        raise SiteMapError(f"nom invalide : {str(nom)[:80]!r}")
    return nom.strip()


def _nombre(nom: str, v, borne: float = COORD_MAX) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) \
            or abs(v) > borne:
        raise SiteMapError(f"{nom} : nombre fini attendu (reçu {v!r})")
    return float(v)


def _verifier(places: Sequence[Place], zones: Sequence[Zone]) -> Tuple[Tuple[Place, ...],
                                                                      Tuple[Zone, ...]]:
    """Vérifie une carte complète ; rend des copies propres ou lève
    SiteMapError."""
    if len(places) > MAX_PLACES or len(zones) > MAX_ZONES:
        raise SiteMapError("trop de lieux ou de zones")
    propres_p, noms = [], set()
    for p in places:
        nom = _nom(p.name)
        if nom in noms:
            raise SiteMapError(f"nom en double : {nom!r}")
        noms.add(nom)
        if p.kind not in PLACE_KINDS:
            raise SiteMapError(f"lieu {nom!r} : type inconnu {p.kind!r}")
        cap = None if p.heading is None else _nombre(f"{nom}.heading", p.heading, 2 * math.pi)
        propres_p.append(Place(nom, _nombre(f"{nom}.x", p.x), _nombre(f"{nom}.y", p.y), cap,
                               p.kind))
    if sum(1 for p in propres_p if p.kind == STATION) > 1:
        raise SiteMapError("une seule station de charge par carte")
    propres_z = []
    for z in zones:
        nom = _nom(z.name)
        if nom in noms:
            raise SiteMapError(f"nom en double : {nom!r}")
        noms.add(nom)
        if z.kind not in ZONE_KINDS:
            raise SiteMapError(f"zone {nom!r} : type inconnu {z.kind!r}")
        if not 3 <= len(z.polygon) <= MAX_VERTICES:
            raise SiteMapError(f"zone {nom!r} : de 3 à {MAX_VERTICES} sommets")
        poly = tuple((_nombre(f"{nom}.x", a), _nombre(f"{nom}.y", b)) for a, b in z.polygon)
        aire = 0.5 * abs(sum(poly[i][0] * poly[(i + 1) % len(poly)][1]
                             - poly[(i + 1) % len(poly)][0] * poly[i][1]
                             for i in range(len(poly))))
        if aire < 1e-4:
            raise SiteMapError(f"zone {nom!r} : polygone plat")
        vmax = None
        if z.kind == SLOW:
            vmax = _nombre(f"{nom}.max_speed_mps", z.max_speed_mps, 1.0)
            if vmax <= 0:
                raise SiteMapError(f"zone lente {nom!r} : max_speed_mps dans ]0, 1] m/s")
        propres_z.append(Zone(nom, z.kind, poly, vmax))
    for p in propres_p:
        for z in propres_z:
            if z.kind == FORBIDDEN and z.contains(p.x, p.y):
                raise SiteMapError(f"le lieu {p.name!r} est dans la zone interdite {z.name!r}")
    return tuple(propres_p), tuple(propres_z)


class SiteMap:
    """Carte du sens du site, versionnée (voir l'en-tête du module).

    path : fichier JSON où elle est enregistrée et relue (facultatif)."""

    def __init__(self, places: Sequence[Place] = (), zones: Sequence[Zone] = (), *,
                 path: Optional[str] = None, author: str = "installation"):
        self._verrou = threading.Lock()
        self._path = path
        self._history: List[SiteMapVersion] = []
        if path and os.path.exists(path):
            self._charger(path)
        else:
            p, z = _verifier(places, zones)
            self._current = SiteMapVersion(1, p, z, author, time.time())
            self._ecrire()

    # ---- lecture -----------------------------------------------------------

    @property
    def current(self) -> SiteMapVersion:
        return self._current

    @property
    def version(self) -> int:
        return self._current.version

    def history(self) -> List[SiteMapVersion]:
        """Versions précédentes, de la plus ancienne à la plus récente."""
        with self._verrou:
            return list(self._history)

    # ---- écriture : administrateur seulement (API de l'OS) -----------------

    def publish(self, places: Sequence[Place], zones: Sequence[Zone], *,
                author: str) -> int:
        """Publie une nouvelle carte complète, vérifiée en entier ; rend son
        numéro de version. Lève SiteMapError sans rien changer si elle est
        refusée."""
        p, z = _verifier(places, zones)
        auteur = _nom(author)
        with self._verrou:
            self._history.append(self._current)
            del self._history[:-HISTORY_MAX]
            self._current = SiteMapVersion(self._current.version + 1, p, z, auteur, time.time())
            self._ecrire()
            return self._current.version

    def rollback(self, version: int, *, author: str) -> int:
        """Republie le contenu d'une version précédente (nouvelle version)."""
        ancienne = next((v for v in self.history() if v.version == version), None)
        if ancienne is None:
            raise SiteMapError(f"version {version} inconnue")
        return self.publish(ancienne.places, ancienne.zones, author=author)

    # ---- fichier -------------------------------------------------------------

    def _ecrire(self) -> None:
        if not self._path:
            return
        doc = {"format": "axis-site-map", "current": self._current.to_json(),
               "history": [v.to_json() for v in self._history]}
        dossier = os.path.dirname(os.path.abspath(self._path))
        fd, tmp = tempfile.mkstemp(dir=dossier, prefix=".site_map.")
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(doc, f, ensure_ascii=False, indent=1)
        os.replace(tmp, self._path)

    @staticmethod
    def _version_de(d: dict) -> SiteMapVersion:
        places = [Place(q["name"], q["x"], q["y"], q.get("heading"), q.get("kind", PLACE))
                  for q in d.get("places", [])]
        zones = [Zone(z["name"], z["kind"], tuple(tuple(s) for s in z["polygon"]),
                      z.get("max_speed_mps")) for z in d.get("zones", [])]
        p, z = _verifier(places, zones)
        return SiteMapVersion(int(d["version"]), p, z, str(d.get("author", "")),
                              float(d.get("published_at", 0.0)))

    def _charger(self, path: str) -> None:
        with open(path, encoding="utf-8") as f:
            doc = json.load(f)
        if doc.get("format") != "axis-site-map":
            raise SiteMapError(f"{path} : pas une carte du sens AXIS-OS")
        self._current = self._version_de(doc["current"])
        self._history = [self._version_de(v) for v in doc.get("history", [])][-HISTORY_MAX:]
