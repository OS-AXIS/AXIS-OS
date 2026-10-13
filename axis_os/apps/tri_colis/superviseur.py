"""
Superviseur en langage naturel pour l'app de tri.

Un opérateur parle au robot comme à un collègue :
    « les colis bleus vont maintenant au poste C »
    « arrête de trier les rouges »
    « fais-moi le point »

Ce que le superviseur peut faire, et rien d'autre :
    - changer la règle couleur -> poste ;
    - suspendre ou reprendre le tri d'une couleur ;
    - produire un rapport de session.

Ce qu'il ne peut pas faire, par construction :
    Il ne reçoit jamais la façade Robot : il n'a aucune référence vers la
    navigation, les bras ou la sécurité. Même un modèle de langage (une IA
    qui produit du texte, ici Claude) qui « hallucine », c'est-à-dire
    invente, un ordre de mouvement n'a aucun chemin pour l'exécuter.
    Demander d'aller plus vite ou de couper la sécurité est refusé, avec
    une explication.

La sortie de l'interpréteur (modèle de langage ou analyseur à règles) est
une donnée non fiable : chaque ordre est validé contre une liste fermée
d'actions, de couleurs et de postes connus avant d'être appliqué.

Le rapport est écrit par du code à partir du bilan, pas par le modèle :
les chiffres présentés à un opérateur ne doivent pas pouvoir être inventés.

Tailles bornées, à la suite d'un audit de sécurité du 10 octobre 2026 : une
consigne de plus de LONGUEUR_MAX_CONSIGNE caractères est refusée sans être
transmise au modèle. Pourquoi : le coût, et la surface offerte à
l'injection d'instructions (un texte conçu pour détourner le modèle de sa
consigne). De même, tout texte venu de l'interpréteur, donc
potentiellement fabriqué, est tronqué avant d'être recopié dans un refus
ou dans le journal.

Tests : test/test_superviseur_tri.py et test/test_superviseur_bornes.py.
"""
from __future__ import annotations

import json
import re
import unicodedata
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Protocol

from axis_os.apps.tri_colis.logique import TriColis

# Liste fermée des actions qu'un ordre peut porter : toute autre est
# refusée.
ACTIONS = ("regle", "suspendre", "reprendre", "retirer_regle", "rapport",
           "hors_perimetre", "incompris")

# Réponse à toute demande de mouvement, de vitesse ou de sécurité.
MOTIF_HORS_PERIMETRE = (
    "je ne commande ni le mouvement ni la sécurité : la vitesse et les "
    "distances de sécurité sont fixées par l'OS, pas par l'app ni par moi")

# Une consigne d'opérateur tient en une phrase ; 500 caractères laissent
# largement la place à plusieurs ordres (« et », « puis »).
LONGUEUR_MAX_CONSIGNE = 500
# Longueur maximale, en caractères, d'un texte venu de l'interpréteur et
# recopié dans un refus ou dans le journal.
LONGUEUR_MAX_ECHO = 60


def _court(x: Any, n: int = LONGUEUR_MAX_ECHO) -> str:
    """Texte non fiable, tronqué et ramené sur une seule ligne, pour un
    message. Une seule ligne : un texte fabriqué ne peut pas glisser de
    fausses lignes dans le journal."""
    t = " ".join(str(x).split())
    return t if len(t) <= n else t[:n - 1] + "…"


@dataclass(frozen=True)
class Ordre:
    """Un ordre validé, prêt à être appliqué."""
    action: str
    couleur: Optional[str] = None
    poste: Optional[str] = None


@dataclass
class Reponse:
    """Résultat d'une consigne : ordres acceptés, motifs des refus, et
    message rendu à l'opérateur."""
    acceptes: List[Ordre] = field(default_factory=list)
    refus: List[str] = field(default_factory=list)
    message: str = ""


class Interpreteur(Protocol):
    """Ce que le superviseur attend d'un interpréteur : transformer la
    consigne d'un opérateur en ordres bruts, qui seront validés ensuite."""

    def interpreter(self, texte: str, contexte: Dict[str, Any]) -> Any:
        """Renvoie une liste de dictionnaires {action, couleur?, poste?,
        motif?} (« ? » : champ facultatif). Le résultat est traité comme
        non fiable."""


def _sans_accents(s: str) -> str:
    """Minuscules sans accents : « Arrête » et « arrete » se reconnaissent
    de la même façon."""
    return "".join(c for c in unicodedata.normalize("NFD", s.lower())
                   if unicodedata.category(c) != "Mn")


# ----------------------------------------------------------------------
# Interpréteur à règles : hors ligne, déterministe, sans modèle.
# ----------------------------------------------------------------------
class InterpreteurRegles:
    """Reconnaît les ordres par mots-clés (expressions régulières), sur le
    texte mis en minuscules et sans accents. Sert aussi d'interpréteur de
    secours quand le modèle de langage est indisponible."""

    # Mots qui trahissent une demande de mouvement ou de sécurité : la
    # demande est refusée, et ce test passe avant tous les autres.
    _HORS = re.compile(r"\b(vite|vitesse|accelere|acceler|rapide|securite|"
                       r"desactive|coupe|ignore|distance|marche|avance|recule|"
                       r"tourne|bras|saute|cours)\b")
    _RAPPORT = re.compile(r"\b(rapport|bilan|point|combien|ou en|resume|statistique)")
    _SUSP = re.compile(r"\b(arrete|stop|stoppe|suspend|suspends|ne .*plus|pause)\b")
    _REPR = re.compile(r"\b(reprend|reprends|reprendre|recommence|relance)\b")
    _RETIRER = re.compile(r"\b(oublie|supprime|retire)\b.*\bregle\b")
    _POSTE = re.compile(r"\bposte\s+([a-z0-9]+)\b|\b(?:vers|en|au|a)\s+([a-z])\b")

    def interpreter(self, texte, contexte):
        """Découpe la consigne en morceaux (virgule, point-virgule, « puis »,
        « et ») et interprète chacun comme un ordre."""
        ordres = []
        for morceau in re.split(r",|;|\bpuis\b|\bet\b", _sans_accents(texte)):
            m = morceau.strip()
            if m:
                ordres.append(self._un(m, contexte))
        # Chaque morceau donne son propre ordre : dans « fais le point et
        # ... », le rapport n'annule pas les autres ordres. Les morceaux
        # incompris sont écartés ; si rien n'est compris, un seul
        # « incompris » est rendu.
        return [o for o in ordres if o["action"] != "incompris"] or \
            [{"action": "incompris"}]

    def _couleur(self, m, couleurs):
        """Couleur nommée dans le morceau : d'abord parmi les couleurs
        connues, sinon parmi des noms de couleur courants. Une couleur
        inconnue sera ensuite refusée par le superviseur, qui la nomme."""
        for c in couleurs:
            if re.search(rf"\b{re.escape(_sans_accents(c))}s?\b", m):
                return c
        mots = re.findall(r"\b(rouge|bleu|vert|jaune|orange|noir|blanc|violet|gris)s?\b", m)
        return mots[0] if mots else None

    def _un(self, m, ctx):
        """Un seul ordre. L'ordre des tests compte : hors périmètre d'abord
        (une demande de mouvement est refusée même si elle nomme une
        couleur), puis rapport, retrait de règle, reprise, suspension, et
        enfin changement de règle."""
        if self._HORS.search(m):
            return {"action": "hors_perimetre", "motif": MOTIF_HORS_PERIMETRE}
        if self._RAPPORT.search(m):
            return {"action": "rapport"}
        couleur = self._couleur(m, ctx.get("couleurs", []))
        if self._RETIRER.search(m):
            return {"action": "retirer_regle", "couleur": couleur}
        if self._REPR.search(m):
            return {"action": "reprendre", "couleur": couleur}
        if self._SUSP.search(m):
            return {"action": "suspendre", "couleur": couleur}
        p = self._POSTE.search(m)
        if couleur and p:
            return {"action": "regle", "couleur": couleur,
                    "poste": (p.group(1) or p.group(2)).upper()}
        return {"action": "incompris"}


# ----------------------------------------------------------------------
# Interpréteur par modèle de langage (Claude). Optionnel : le paquet
# `anthropic` n'est importé que si on l'utilise.
# ----------------------------------------------------------------------
# Consigne système envoyée au modèle ; les champs entre accolades sont
# remplacés par le contexte, écrit en JSON (un format texte de données
# structurées). Elle rappelle les limites au modèle,
# mais la sécurité ne repose pas sur elle : chaque ordre rendu est validé
# par le Superviseur.
_CONSIGNE = """Tu traduis les instructions d'un opérateur d'entrepôt en ordres
pour une application de tri de colis. Réponds UNIQUEMENT par un tableau JSON.

Actions permises :
  {"action":"regle","couleur":C,"poste":P}   colis de couleur C vers poste P
  {"action":"suspendre","couleur":C}         ne plus trier la couleur C
  {"action":"reprendre","couleur":C}         reprendre le tri de C
  {"action":"retirer_regle","couleur":C}     supprimer la règle de C
  {"action":"rapport"}                       l'opérateur veut un bilan
  {"action":"hors_perimetre","motif":M}      demande de mouvement, de vitesse,
                                             de sécurité ou autre : REFUSE
  {"action":"incompris"}                     tu ne comprends pas

Tu ne commandes jamais le mouvement ni la sécurité du robot.
Couleurs connues : {couleurs}. Postes connus : {postes}.
Règles actuelles : {regles}. Couleurs suspendues : {suspendues}."""


class InterpreteurClaude:
    """Interprète les consignes avec un modèle de langage, par l'API
    d'Anthropic (l'interface d'accès en ligne à ses modèles)."""

    def __init__(self, client=None, modele: str = "claude-haiku-4-5-20251001",
                 max_tokens: int = 400):
        """client : client de l'API déjà construit (les tests en passent un
        factice) ; à défaut, il est créé ici. max_tokens : longueur
        maximale de la réponse du modèle, en jetons (morceaux de mots)."""
        if client is None:
            import anthropic                    # dépendance optionnelle
            client = anthropic.Anthropic()
        self.client, self.modele, self.max_tokens = client, modele, max_tokens

    def interpreter(self, texte, contexte):
        """Ordres bruts extraits de la réponse du modèle : le texte compris
        entre le premier « [ » et le dernier « ] » est lu comme du JSON.
        Une réponse illisible devient « incompris »."""
        systeme = _CONSIGNE
        for k in ("couleurs", "postes", "regles", "suspendues"):
            systeme = systeme.replace("{" + k + "}", json.dumps(
                contexte.get(k), ensure_ascii=False))
        rep = self.client.messages.create(
            model=self.modele, max_tokens=self.max_tokens, system=systeme,
            messages=[{"role": "user", "content": texte}])
        brut = "".join(getattr(b, "text", "") for b in rep.content).strip()
        debut, fin = brut.find("["), brut.rfind("]")
        if debut < 0 or fin < debut:
            return [{"action": "incompris"}]
        try:
            return json.loads(brut[debut:fin + 1])
        except json.JSONDecodeError:
            return [{"action": "incompris"}]


# ----------------------------------------------------------------------
class Superviseur:
    """Reçoit les consignes de l'opérateur et les applique à la logique de
    tri, après validation."""

    def __init__(self, tri: TriColis, postes: Iterable[str],
                 couleurs: Iterable[str], interpreteur: Interpreteur,
                 secours: Optional[Interpreteur] = None,
                 max_ordres: int = 5):
        """secours : interpréteur utilisé si le principal lève une
        exception (par défaut, l'interpréteur à règles). max_ordres : une
        consigne qui en contient davantage est refusée en entier."""
        # Seule la logique de tri est gardée, jamais la façade Robot : le
        # superviseur n'a ainsi aucun chemin vers le mouvement (voir l'en-tête
        # du module).
        self._tri = tri
        self.postes = {p.upper() for p in postes}
        self.couleurs = set(couleurs)
        self.interpreteur = interpreteur
        self.secours = secours or InterpreteurRegles()
        self.max_ordres = max_ordres
        self.historique: List[tuple] = []     # (texte, Reponse)

    def _contexte(self) -> Dict[str, Any]:
        """Ce que l'interpréteur reçoit : couleurs connues, postes de
        destination possibles (sans le poste de prise), règles actuelles et
        couleurs suspendues."""
        return {"couleurs": sorted(self.couleurs),
                "postes": sorted(self.postes - {self._tri.cfg.poste_prise.upper()}),
                "regles": dict(self._tri.cfg.regles),
                "suspendues": list(self._tri.bilan.couleurs_suspendues)}

    def ordonner(self, texte: str) -> Reponse:
        """Traite une consigne de l'opérateur : interprétation, validation
        de chaque ordre (_valider), puis application des ordres valides.
        Tout est consigné dans l'historique et le journal."""
        if not isinstance(texte, str) or len(texte) > LONGUEUR_MAX_CONSIGNE:
            # Refusée avant l'interpréteur : rien n'est envoyé au modèle.
            longueur = len(texte) if isinstance(texte, str) else "?"
            rep = Reponse(refus=[f"consigne trop longue ou illisible ({longueur} "
                                 f"caractères, maximum {LONGUEUR_MAX_CONSIGNE})"])
            rep.message = f"Refusé : {rep.refus[0]}."
            self.historique.append((_court(texte, 200), rep))
            self._tri._log(f"superviseur : {rep.message}")
            return rep
        ctx = self._contexte()
        try:
            brut = self.interpreteur.interpreter(texte, ctx)
        except Exception as e:
            # Interpréteur en erreur (modèle injoignable, par exemple) : on ne
            # reste pas muet, l'interpréteur de secours prend le relais.
            self._tri._log(f"superviseur : interpréteur indisponible "
                           f"({_court(repr(e), 120)}), secours")
            brut = self.secours.interpreter(texte, ctx)
        rep = Reponse()
        # La sortie de l'interpréteur n'est pas fiable : un ordre seul est
        # mis en liste, toute autre forme devient « incompris ».
        if isinstance(brut, dict):
            brut = [brut]
        if not isinstance(brut, list):
            brut = [{"action": "incompris"}]
        if len(brut) > self.max_ordres:
            # Trop d'ordres d'un coup : tout est refusé, plutôt que d'en
            # appliquer une partie.
            rep.refus.append(f"trop d'ordres à la fois ({len(brut)}), "
                             f"maximum {self.max_ordres}")
            brut = []
        messages = []
        for o in brut:
            ordre, erreur = self._valider(o)
            if erreur:
                rep.refus.append(erreur)
                continue
            messages.append(self._appliquer(ordre))
            rep.acceptes.append(ordre)
        rep.message = " ".join(messages + [f"Refusé : {r}." for r in rep.refus]) \
            or "Je n'ai rien compris d'applicable."
        self.historique.append((texte, rep))
        self._tri._log(f"superviseur « {texte} » -> {rep.message}")
        return rep

    def _valider(self, o: Any):
        """Vérifie un ordre brut contre les listes fermées (actions,
        couleurs, postes connus). Rend (Ordre, None) s'il est valide,
        (None, motif du refus) sinon. Un texte venu de l'interpréteur n'est
        recopié dans le motif que tronqué (_court)."""
        if not isinstance(o, dict):
            return None, "réponse de l'interpréteur mal formée"
        action = o.get("action")
        if not isinstance(action, str) or action not in ACTIONS:
            return None, f"action inconnue « {_court(action)} »"
        if action == "hors_perimetre":
            return None, MOTIF_HORS_PERIMETRE
        if action == "incompris":
            return None, "instruction non comprise — reformulez"
        if action == "rapport":
            return Ordre("rapport"), None
        couleur = o.get("couleur")
        if not isinstance(couleur, str) or couleur not in self.couleurs:
            return None, f"couleur inconnue « {_court(couleur)} »"
        if action == "regle":
            poste = o.get("poste")
            poste = poste.upper() if isinstance(poste, str) else None
            if poste not in self.postes:
                return None, f"poste inconnu « {_court(o.get('poste'))} »"
            if poste == self._tri.cfg.poste_prise.upper():
                return None, "le poste de prise ne peut pas être une destination"
            return Ordre("regle", couleur, poste), None
        if action in ("reprendre", "retirer_regle") and \
                couleur not in self._tri.cfg.regles:
            return None, f"aucune règle pour les colis {couleur}"
        return Ordre(action, couleur), None

    def _appliquer(self, o: Ordre) -> str:
        """Applique un ordre déjà validé ; rend la phrase de réponse."""
        cfg, bilan = self._tri.cfg, self._tri.bilan
        susp = bilan.couleurs_suspendues
        if o.action == "rapport":
            return self.rapport()
        if o.action == "regle":
            ancien = cfg.regles.get(o.couleur)
            cfg.regles[o.couleur] = o.poste
            # Une nouvelle destination lève une suspension (souvent due à un
            # poste inaccessible) : c'est précisément le but de l'ordre.
            if o.couleur in susp:
                susp.remove(o.couleur)
            en_main = self._tri.colis is not None and \
                self._tri.colis.couleur == o.couleur and self._tri.destination
            suite = (" Le colis en main termine vers l'ancien poste."
                     if en_main else "")
            return (f"Colis {o.couleur} : poste {ancien or '—'} -> {o.poste}."
                    + suite)
        if o.action == "suspendre":
            if o.couleur not in susp:
                susp.append(o.couleur)
            return f"Tri des colis {o.couleur} suspendu."
        if o.action == "reprendre":
            if o.couleur in susp:
                susp.remove(o.couleur)
            return f"Tri des colis {o.couleur} repris (poste {cfg.regles[o.couleur]})."
        if o.action == "retirer_regle":
            del cfg.regles[o.couleur]
            if o.couleur in susp:
                susp.remove(o.couleur)
            return f"Règle des colis {o.couleur} supprimée : ils ne seront plus pris."
        raise AssertionError(o)

    def rapport(self) -> str:
        """Rapport de session, écrit par du code à partir du bilan : aucun
        chiffre ne peut être inventé."""
        b, t = self._tri.bilan, self._tri.duree_ecoulee()
        detail = ", ".join(f"{n} {c}" for c, n in sorted(b.par_couleur.items())) or "aucun"
        regles = ", ".join(f"{c}→{p}" for c, p in sorted(self._tri.cfg.regles.items()))
        txt = (f"Bilan à {int(t // 60)} min {int(t % 60):02d} s : "
               f"{b.colis_tries} colis triés ({detail}), "
               f"taux de réussite {100 * b.taux_reussite():.0f} %, "
               f"{b.colis_abandonnes} signalé(s), "
               f"{b.temps_arret_securite_s:.0f} s d'arrêts de sécurité. "
               f"Règles : {regles}.")
        if b.couleurs_suspendues:
            txt += f" Suspendues : {', '.join(b.couleurs_suspendues)}."
        if b.alertes:
            txt += f" Dernière alerte : {b.alertes[-1]}"
        return txt
