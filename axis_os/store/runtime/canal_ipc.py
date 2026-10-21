"""
AXIS-OS — Canal IPC du pont entre une app et l'OS
==================================================
Transport et validation des messages échangés entre le processus d'une
app et celui de l'OS (voir hal_bridge.py et processus_app.py).

Pourquoi ce module : le pont transportait des objets pickle (le format
de Python qui transforme un objet en octets et inversement), et
désérialiser un pickle peut exécuter du code. Un objet fabriqué par une
app faisait donc tourner du code de son choix dans le processus de l'OS
(constat 6 de docs/isolation_apps.md). Ici ne circulent que des octets :

  - chaque message est une trame : 4 octets de longueur (gros-boutiste),
    puis un objet JSON encodé en UTF-8 ;
  - une trame plus longue que TAILLE_MAX_TRAME est refusée sans être lue,
    et le canal est alors fermé : on ne peut pas retrouver le début de la
    trame suivante sans lire celle-ci en entier ;
  - le JSON est décodé strictement : pas de NaN ni d'Infinity, pas de
    nombre non fini, pas de clé en double, pas d'entier démesuré, une
    profondeur d'imbrication bornée, un objet (dict) au premier niveau ;
  - chaque type de message est ensuite vérifié par un schéma (classes
    Chaine, Nombre, Objet… ci-dessous) : un champ inconnu, absent ou du
    mauvais type fait refuser le message.

Aucun objet Python ne traverse le canal, dans un sens comme dans
l'autre : seulement des dicts, listes, chaînes, nombres, booléens et
None, reconstruits par json.loads.

Licence : Apache 2.0, comme tout le dépôt (voir LICENSE).
"""

from __future__ import annotations

import json
import math
import socket
import struct
import threading
import time
from typing import Any, Dict, Optional

# Une trame de 1 Mio suffit largement : la plus grosse réponse normale
# (get_full_state d'un G1, 29 articulations) fait moins de 10 Kio.
TAILLE_MAX_TRAME = 1 << 20
PROFONDEUR_MAX = 32
# Au-delà, un entier n'est plus un nombre physique plausible ; JSON ne
# borne pas les entiers, et Python les accepte de toute taille.
ENTIER_MAX = 2 ** 63
_ENTETE = struct.Struct(">I")
DELAI_TRAME_S = 5.0


class MessageRefuse(Exception):
    """Message reçu qui n'est pas un JSON strict ou ne suit pas son
    schéma. Le canal reste utilisable : la trame a été lue en entier."""


class CanalCorrompu(Exception):
    """Trame trop longue ou tronquée : le canal ne peut plus être lu et
    doit être fermé."""


class CanalFerme(Exception):
    """L'autre extrémité a fermé le canal."""


# ─────────────────────────────────────────────────────────────────
#  JSON strict
# ─────────────────────────────────────────────────────────────────

def _constante_interdite(nom: str):
    raise ValueError(f"constante JSON interdite : {nom}")


def _flottant_fini(texte: str) -> float:
    valeur = float(texte)
    if not math.isfinite(valeur):
        raise ValueError(f"nombre non fini : {texte[:20]}")
    return valeur


def _entier_borne(texte: str) -> int:
    # Le nombre de chiffres est vérifié avant la conversion, qui coûte
    # cher sur un entier de plusieurs milliers de chiffres.
    if len(texte.lstrip("-")) > 19:
        raise ValueError("entier trop long")
    valeur = int(texte)
    if abs(valeur) > ENTIER_MAX:
        raise ValueError("entier hors bornes")
    return valeur


def _sans_doublon(paires) -> Dict[str, Any]:
    objet: Dict[str, Any] = {}
    for cle, valeur in paires:
        if cle in objet:
            raise ValueError(f"clé en double : {cle[:40]!r}")
        objet[cle] = valeur
    return objet


def _profondeur(valeur: Any) -> int:
    """Profondeur d'imbrication, calculée sans récursion."""
    pire, pile = 0, [(valeur, 1)]
    while pile:
        v, p = pile.pop()
        if isinstance(v, dict):
            pire = max(pire, p)
            pile.extend((x, p + 1) for x in v.values())
        elif isinstance(v, list):
            pire = max(pire, p)
            pile.extend((x, p + 1) for x in v)
    return pire


def decoder(octets: bytes) -> Dict[str, Any]:
    """Octets reçus -> dict, ou MessageRefuse."""
    try:
        texte = octets.decode("utf-8")
        valeur = json.loads(texte, parse_constant=_constante_interdite,
                            parse_float=_flottant_fini, parse_int=_entier_borne,
                            object_pairs_hook=_sans_doublon)
    except (UnicodeDecodeError, ValueError, RecursionError) as exc:
        raise MessageRefuse(f"JSON invalide : {type(exc).__name__}: {str(exc)[:120]}") from None
    if not isinstance(valeur, dict):
        raise MessageRefuse("le message doit être un objet JSON")
    if _profondeur(valeur) > PROFONDEUR_MAX:
        raise MessageRefuse(f"imbrication trop profonde (> {PROFONDEUR_MAX})")
    return valeur


def encoder(message: Dict[str, Any], taille_max: int = TAILLE_MAX_TRAME) -> bytes:
    """dict -> trame (en-tête de longueur + JSON). Lève ValueError si le
    message n'est pas du JSON pur ou dépasse taille_max."""
    if not isinstance(message, dict):
        raise ValueError("un message doit être un dict")
    # allow_nan=False refuse NaN et l'infini ; json.dumps refuse de lui-même
    # tout objet qui n'est pas un type JSON de base.
    corps = json.dumps(message, ensure_ascii=False, allow_nan=False,
                       separators=(",", ":")).encode("utf-8")
    if len(corps) > taille_max:
        raise ValueError(f"message de {len(corps)} octets, au-delà de {taille_max}")
    return _ENTETE.pack(len(corps)) + corps


# ─────────────────────────────────────────────────────────────────
#  Transport : une socket Unix, des trames
# ─────────────────────────────────────────────────────────────────

class Canal:
    """Une extrémité du pont : envoie et reçoit des dicts JSON.

    Les envois sont protégés par un verrou (plusieurs threads de l'app
    peuvent appeler le pont) ; les réceptions ne doivent venir que d'un
    seul thread à la fois.
    """

    def __init__(self, sock: socket.socket, taille_max: int = TAILLE_MAX_TRAME):
        self._sock = sock
        self._taille_max = taille_max
        self._verrou_envoi = threading.Lock()
        self._ferme = False

    @classmethod
    def depuis_descripteur(cls, fd: int, **kw) -> "Canal":
        return cls(socket.socket(fileno=fd), **kw)

    def envoyer(self, message: Dict[str, Any]) -> None:
        trame = encoder(message, self._taille_max)
        with self._verrou_envoi:
            try:
                self._sock.sendall(trame)
            except OSError as exc:
                raise CanalFerme(str(exc)) from None

    def _lire_exactement(self, n: int, echeance: Optional[float]) -> bytes:
        """Lit exactement n octets avant l'échéance (time.monotonic()), ou
        sans limite si echeance est None."""
        morceaux, reste = [], n
        while reste:
            if echeance is None:
                self._sock.settimeout(None)
            else:
                attente = echeance - time.monotonic()
                if attente <= 0:
                    raise TimeoutError
                self._sock.settimeout(attente)
            try:
                bloc = self._sock.recv(min(reste, 65536))
            except socket.timeout:
                raise TimeoutError from None
            except OSError as exc:
                raise CanalFerme(str(exc)) from None
            if not bloc:
                if reste == n:
                    raise CanalFerme("fin de flux")
                raise CanalCorrompu("trame tronquée")
            morceaux.append(bloc)
            reste -= len(bloc)
        return b"".join(morceaux)

    def recevoir(self, timeout: Optional[float] = None) -> Dict[str, Any]:
        """Attend une trame et la décode.

        TimeoutError si rien n'arrive dans le délai (rien n'est consommé) ;
        MessageRefuse si la trame est lue mais invalide (le canal reste
        utilisable) ; CanalCorrompu si la trame annoncée dépasse la taille
        maximale ou n'arrive pas en entier dans DELAI_TRAME_S (le canal est
        alors fermé) ; CanalFerme si l'autre extrémité est partie."""
        echeance = None if timeout is None else time.monotonic() + timeout
        premier = self._lire_exactement(1, echeance)
        # Une trame commencée doit arriver en entier, et vite : sans cette
        # limite, une app qui enverrait un octet par minute bloquerait le
        # pont indéfiniment.
        limite = time.monotonic() + DELAI_TRAME_S
        try:
            entete = premier + self._lire_exactement(_ENTETE.size - 1, limite)
            (longueur,) = _ENTETE.unpack(entete)
            if longueur > self._taille_max:
                raise CanalCorrompu(f"trame annoncée de {longueur} octets, "
                                    f"au-delà de {self._taille_max}")
            corps = self._lire_exactement(longueur, limite)
        except TimeoutError:
            self.fermer()
            raise CanalCorrompu(f"trame incomplète après {DELAI_TRAME_S} s") from None
        except CanalCorrompu:
            self.fermer()
            raise
        return decoder(corps)

    def fermer(self) -> None:
        if self._ferme:
            return
        self._ferme = True
        try:
            self._sock.close()
        except OSError:
            pass


# ─────────────────────────────────────────────────────────────────
#  Schémas
# ─────────────────────────────────────────────────────────────────

class Schema:
    """Vérifie une valeur déjà décodée ; lève MessageRefuse sinon."""

    def verifier(self, valeur: Any, chemin: str = "$") -> Any:
        raise NotImplementedError

    @staticmethod
    def _refus(chemin: str, raison: str):
        raise MessageRefuse(f"{chemin} : {raison}")


class Chaine(Schema):
    def __init__(self, max_len: int = 256):
        self.max_len = max_len

    def verifier(self, valeur, chemin="$"):
        if not isinstance(valeur, str):
            self._refus(chemin, "chaîne attendue")
        if len(valeur) > self.max_len:
            self._refus(chemin, f"chaîne de plus de {self.max_len} caractères")
        return valeur


class Nombre(Schema):
    """Nombre fini (int ou float, jamais bool), borné si demandé."""

    def __init__(self, mini: Optional[float] = None, maxi: Optional[float] = None):
        self.mini, self.maxi = mini, maxi

    def verifier(self, valeur, chemin="$"):
        if isinstance(valeur, bool) or not isinstance(valeur, (int, float)):
            self._refus(chemin, "nombre attendu")
        if not math.isfinite(valeur):
            self._refus(chemin, "nombre non fini")
        if self.mini is not None and valeur < self.mini:
            self._refus(chemin, f"inférieur à {self.mini}")
        if self.maxi is not None and valeur > self.maxi:
            self._refus(chemin, f"supérieur à {self.maxi}")
        return valeur


class Entier(Nombre):
    def verifier(self, valeur, chemin="$"):
        if isinstance(valeur, bool) or not isinstance(valeur, int):
            self._refus(chemin, "entier attendu")
        return super().verifier(valeur, chemin)


class Booleen(Schema):
    def verifier(self, valeur, chemin="$"):
        if not isinstance(valeur, bool):
            self._refus(chemin, "booléen attendu")
        return valeur


class Facultatif(Schema):
    """La valeur peut aussi être null (None)."""

    def __init__(self, schema: Schema):
        self.schema = schema

    def verifier(self, valeur, chemin="$"):
        return None if valeur is None else self.schema.verifier(valeur, chemin)


class UnParmi(Schema):
    def __init__(self, *choix: str):
        self.choix = frozenset(choix)

    def verifier(self, valeur, chemin="$"):
        if not isinstance(valeur, str) or valeur not in self.choix:
            self._refus(chemin, f"valeur hors de {sorted(self.choix)}")
        return valeur


class Liste(Schema):
    def __init__(self, element: Schema, max_len: int = 256):
        self.element, self.max_len = element, max_len

    def verifier(self, valeur, chemin="$"):
        if not isinstance(valeur, list):
            self._refus(chemin, "liste attendue")
        if len(valeur) > self.max_len:
            self._refus(chemin, f"plus de {self.max_len} éléments")
        return [self.element.verifier(v, f"{chemin}[{i}]") for i, v in enumerate(valeur)]


class Dictionnaire(Schema):
    """Dict aux clés libres (chaînes bornées) et aux valeurs d'un même
    schéma."""

    def __init__(self, valeur: Schema, max_len: int = 256, cle: Schema = Chaine(128)):
        self.valeur, self.max_len, self.cle = valeur, max_len, cle

    def verifier(self, valeur, chemin="$"):
        if not isinstance(valeur, dict):
            self._refus(chemin, "objet attendu")
        if len(valeur) > self.max_len:
            self._refus(chemin, f"plus de {self.max_len} entrées")
        return {self.cle.verifier(k, f"{chemin}.<clé>"): self.valeur.verifier(v, f"{chemin}.{k}")
                for k, v in valeur.items()}


class Objet(Schema):
    """Dict aux champs fixés. Tout champ inconnu fait refuser le message ;
    un champ de `facultatifs` absent prend la valeur None."""

    def __init__(self, champs: Dict[str, Schema],
                 facultatifs: Optional[Dict[str, Schema]] = None):
        self.champs = champs
        self.facultatifs = facultatifs or {}

    def verifier(self, valeur, chemin="$"):
        if not isinstance(valeur, dict):
            self._refus(chemin, "objet attendu")
        inconnus = set(valeur) - set(self.champs) - set(self.facultatifs)
        if inconnus:
            self._refus(chemin, f"champ(s) inattendu(s) : {sorted(inconnus)[:5]}")
        sortie = {}
        for nom, schema in self.champs.items():
            if nom not in valeur:
                self._refus(chemin, f"champ manquant : {nom}")
            sortie[nom] = schema.verifier(valeur[nom], f"{chemin}.{nom}")
        for nom, schema in self.facultatifs.items():
            v = valeur.get(nom)
            sortie[nom] = None if v is None else schema.verifier(v, f"{chemin}.{nom}")
        return sortie


class JsonLibre(Schema):
    """N'importe quelle valeur JSON (déjà bornée par le décodage strict).
    Pour les données que l'app confie à l'OS sans qu'il les interprète :
    sa mémoire (remember), les métriques d'une mission."""

    def verifier(self, valeur, chemin="$"):
        return valeur
