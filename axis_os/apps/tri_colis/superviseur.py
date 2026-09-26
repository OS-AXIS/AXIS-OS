"""
Superviseur en langage naturel pour l'app de tri.

Un opérateur parle au robot comme à un collègue :
    « les colis bleus vont maintenant au poste C »
    « arrête de trier les rouges »
    « fais-moi le point »

CE QUE LE SUPERVISEUR PEUT FAIRE — et rien d'autre :
    - changer la règle couleur -> poste ;
    - suspendre / reprendre le tri d'une couleur ;
    - produire un rapport de session.

CE QU'IL NE PEUT PAS FAIRE, PAR CONSTRUCTION :
    Il ne reçoit jamais la façade Robot : il n'a aucune référence vers la
    navigation, les bras ou la sécurité. Même un modèle de langage qui
    « hallucine » un ordre de mouvement n'a aucun chemin pour l'exécuter.
    Demander d'aller plus vite ou de couper la sécurité est refusé avec
    une explication.

La sortie de l'interpréteur (modèle de langage ou analyseur à règles) est
une donnée NON FIABLE : chaque ordre est validé contre une liste fermée
d'actions, de couleurs et de postes connus avant d'être appliqué.

Le rapport est écrit par du code à partir du bilan, pas par le modèle :
les chiffres présentés à un opérateur ne doivent pas pouvoir être inventés.
"""
from __future__ import annotations

import json
import re
import unicodedata
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Protocol

from axis_os.apps.tri_colis.logique import TriColis

ACTIONS = ("regle", "suspendre", "reprendre", "retirer_regle", "rapport",
           "hors_perimetre", "incompris")

MOTIF_HORS_PERIMETRE = (
    "je ne commande ni le mouvement ni la sécurité : la vitesse et les "
    "distances de sécurité sont fixées par l'OS, pas par l'app ni par moi")


@dataclass(frozen=True)
class Ordre:
    action: str
    couleur: Optional[str] = None
    poste: Optional[str] = None


@dataclass
class Reponse:
    acceptes: List[Ordre] = field(default_factory=list)
    refus: List[str] = field(default_factory=list)
    message: str = ""


class Interpreteur(Protocol):
    def interpreter(self, texte: str, contexte: Dict[str, Any]) -> Any:
        """Renvoie une liste de dicts {action, couleur?, poste?, motif?}.
        Le résultat est traité comme non fiable."""


def _sans_accents(s: str) -> str:
    return "".join(c for c in unicodedata.normalize("NFD", s.lower())
                   if unicodedata.category(c) != "Mn")


# ----------------------------------------------------------------------
# Interpréteur à règles : hors ligne, déterministe, sans modèle.
# ----------------------------------------------------------------------
class InterpreteurRegles:
    _HORS = re.compile(r"\b(vite|vitesse|accelere|acceler|rapide|securite|"
                       r"desactive|coupe|ignore|distance|marche|avance|recule|"
                       r"tourne|bras|saute|cours)\b")
    _RAPPORT = re.compile(r"\b(rapport|bilan|point|combien|ou en|resume|statistique)")
    _SUSP = re.compile(r"\b(arrete|stop|stoppe|suspend|suspends|ne .*plus|pause)\b")
    _REPR = re.compile(r"\b(reprend|reprends|reprendre|recommence|relance)\b")
    _RETIRER = re.compile(r"\b(oublie|supprime|retire)\b.*\bregle\b")
    _POSTE = re.compile(r"\bposte\s+([a-z0-9]+)\b|\b(?:vers|en|au|a)\s+([a-z])\b")

    def interpreter(self, texte, contexte):
        ordres = []
        for morceau in re.split(r",|;|\bpuis\b|\bet\b", _sans_accents(texte)):
            m = morceau.strip()
            if m:
                ordres.append(self._un(m, contexte))
        # « fais le point et ... » : un rapport n'annule pas les autres ordres
        return [o for o in ordres if o["action"] != "incompris"] or \
            [{"action": "incompris"}]

    def _couleur(self, m, couleurs):
        for c in couleurs:
            if re.search(rf"\b{re.escape(_sans_accents(c))}s?\b", m):
                return c
        mots = re.findall(r"\b(rouge|bleu|vert|jaune|orange|noir|blanc|violet|gris)s?\b", m)
        return mots[0] if mots else None

    def _un(self, m, ctx):
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
    def __init__(self, client=None, modele: str = "claude-haiku-4-5-20251001",
                 max_tokens: int = 400):
        if client is None:
            import anthropic                    # dépendance optionnelle
            client = anthropic.Anthropic()
        self.client, self.modele, self.max_tokens = client, modele, max_tokens

    def interpreter(self, texte, contexte):
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
    def __init__(self, tri: TriColis, postes: Iterable[str],
                 couleurs: Iterable[str], interpreteur: Interpreteur,
                 secours: Optional[Interpreteur] = None,
                 max_ordres: int = 5):
        # NB : seulement la logique de tri — jamais la façade Robot.
        self._tri = tri
        self.postes = {p.upper() for p in postes}
        self.couleurs = set(couleurs)
        self.interpreteur = interpreteur
        self.secours = secours or InterpreteurRegles()
        self.max_ordres = max_ordres
        self.historique: List[tuple] = []     # (texte, Reponse)

    def _contexte(self) -> Dict[str, Any]:
        return {"couleurs": sorted(self.couleurs),
                "postes": sorted(self.postes - {self._tri.cfg.poste_prise.upper()}),
                "regles": dict(self._tri.cfg.regles),
                "suspendues": list(self._tri.bilan.couleurs_suspendues)}

    def ordonner(self, texte: str) -> Reponse:
        ctx = self._contexte()
        try:
            brut = self.interpreteur.interpreter(texte, ctx)
        except Exception as e:      # modèle injoignable : on ne reste pas muet
            self._tri._log(f"superviseur : interpréteur indisponible ({e!r}), secours")
            brut = self.secours.interpreter(texte, ctx)
        rep = Reponse()
        if isinstance(brut, dict):
            brut = [brut]
        if not isinstance(brut, list):
            brut = [{"action": "incompris"}]
        if len(brut) > self.max_ordres:
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
        if not isinstance(o, dict):
            return None, "réponse de l'interpréteur mal formée"
        action = o.get("action")
        if action not in ACTIONS:
            return None, f"action inconnue « {action} »"
        if action == "hors_perimetre":
            return None, MOTIF_HORS_PERIMETRE
        if action == "incompris":
            return None, "instruction non comprise — reformulez"
        if action == "rapport":
            return Ordre("rapport"), None
        couleur = o.get("couleur")
        if not isinstance(couleur, str) or couleur not in self.couleurs:
            return None, f"couleur inconnue « {couleur} »"
        if action == "regle":
            poste = o.get("poste")
            poste = poste.upper() if isinstance(poste, str) else None
            if poste not in self.postes:
                return None, f"poste inconnu « {o.get('poste')} »"
            if poste == self._tri.cfg.poste_prise.upper():
                return None, "le poste de prise ne peut pas être une destination"
            return Ordre("regle", couleur, poste), None
        if action in ("reprendre", "retirer_regle") and \
                couleur not in self._tri.cfg.regles:
            return None, f"aucune règle pour les colis {couleur}"
        return Ordre(action, couleur), None

    def _appliquer(self, o: Ordre) -> str:
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
        """Écrit par du code, à partir du bilan : aucun chiffre inventé."""
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
