"""
AXIS-OS — Robotics Play Store Runtime
=======================================
Moteur d'exécution du Play Store côté robot. Le Play Store est la
boutique d'apps du robot : des développeurs tiers y publient des apps
(tri de colis, inventaire d'entrepôt…) que le robot installe puis
exécute.

Ce module gère, sur le robot :
  - l'installation, précédée de toutes les vérifications : manifeste
    (la fiche que chaque app déclare : identité, version, permissions,
    matériel requis, prix, quotas), compatibilité avec le SDK (le kit de
    développement des apps), signature du code, niveau de protection des
    permissions, capacités du robot ;
  - le cycle de vie : démarrage, missions, arrêt, mise à jour ;
  - l'exécution de chaque appel à l'app dans une sandbox (un « bac à
    sable » : un processus séparé, avec des quotas de CPU, de mémoire
    et de durée ; voir sandbox.py) ;
  - la facturation : la plateforme prélève une commission de 30 % sur
    le prix payé par le client, le développeur reçoit les 70 % restants.

Limite connue : la sandbox protège contre une app buggée, pas encore
contre une app délibérément hostile. Le constat et le plan de correction
sont dans docs/isolation_apps.md et SECURITY.md (« Limites connues ») ;
en attendant, ne faire tourner sur un vrai robot que des apps dont on
connaît et relit le code.

Licence : Apache 2.0, comme tout le dépôt (voir LICENSE).
"""

from __future__ import annotations
import copy
from dataclasses import dataclass, field, fields, replace
from typing import Callable, Optional, Dict, Any, List, Union
import hashlib
import math
import os
import time
import uuid
import traceback

from axis_os import SemVer
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.watchdog.system import WatchdogSystem
from axis_os.hal.core.capabilities import RobotCapability
from axis_os.store.sdk.manifest import (
    AppManifest, Permission, PricingModel, Capability, ProtectionLevel,
    get_protection_level,
)
from axis_os.store.sdk.app_base import (
    RobotApp, RobotContext, MissionResult, PermissionError, EstopActiveError,
    _RuntimeSessionToken,
)
from axis_os.store.runtime.sandbox import (
    SandboxRunner, SandboxResult, SandboxState, SandboxErrorKind, _try_import_psutil,
)
from axis_os.store.runtime.profil import (
    PLAFONDS_QUOTAS, Profil, ecrire_registre_apps, lire_cles_approuvees, lire_registre_apps,
)
from axis_os.store.runtime.hal_bridge import HALBridgeServer, HALBridgeClient
from axis_os.store.runtime.processus_app import (
    AppNonRechargeable, etat_json, rendre_os_non_inspectable, spec_depuis_instance,
)
from axis_os.services.memory.service import MemoryService
from axis_os.store.sdk.signing import SignatureError, message_paquet, public_key_fingerprint
import threading


# ── Confirmation des permissions DANGEROUS ────────────────────────
#
# Chaque permission a un niveau de protection (ProtectionLevel, voir
# axis_os/store/sdk/manifest.py). Le niveau DANGEROUS regroupe les
# permissions qui donnent à l'app un pouvoir physique sur le robot (bouger
# ses bras ou ses jambes, le faire marcher…) ; il promet une confirmation
# humaine avant l'installation.
#
# Pourquoi ce code : un audit de sécurité (2026-10-10) a montré que cette
# promesse n'était pas tenue. install() ne demandait rien, et une app
# déclarant ACTUATORS_WRITE_ALL (commander tout le corps) ou
# ACTUATORS_AI_DRIVEN (mouvements dont les consignes viennent d'un modèle
# d'IA) s'installait sans que personne n'ait accepté qu'elle bouge le
# robot.
#
# Règle désormais : refus par défaut. Une permission DANGEROUS n'est
# accordée que si la fonction `confirmer_permissions` fournie au runtime
# répond exactement True ; il n'existe aucun accord implicite. Essais :
# test/test_permissions_dangereuses.py.

ConfirmationPermissions = Callable[..., bool]


@dataclass(frozen=True)
class DetailsConfirmation:
    """Ce que l'humain doit voir, en plus du manifeste, avant d'accorder des
    permissions DANGEROUS : qui a signé l'app et sous quel profil elle
    s'installe. Les quotas sont dans manifest.quota, déjà vérifiés contre
    les plafonds du profil.

    Un confirmateur qui porte l'attribut `recoit_details = True` est appelé
    avec ces détails en troisième argument ; c'est obligatoire en profil
    PRODUCTION, pour que la clé signataire et les quotas soient montrés."""
    profil:         str
    signataire:     Optional[str]   # empreinte de la clé vérifiée ; None si non signée
    nom_signataire: Optional[str]   # nom inscrit dans le fichier des clés approuvées
    fichier_cles:   Optional[str]   # chemin de ce fichier, s'il y en a un
    liaison:        Optional[str] = None  # ce que dit le registre des apps de cet app_id
    prix:           Optional[str] = None  # ce que paiera le client (decrire_prix)
    prix_precedent: Optional[str] = None  # prix de la version installée, s'il change


MOTIF_ARRET_URGENCE = "Mission interrompue par arrêt d'urgence (e-stop)"

_UNITES = {"max_cpu_percent": "% d'un cœur", "max_memory_mb": "Mo", "max_runtime_s": "s"}


def decrire_quotas(q: Any) -> List[str]:
    """Les quotas d'une app, en clair, tels que la sandbox les applique :
    sans psutil (permis hors production), CPU et mémoire vive ne le sont
    pas, et l'humain doit le lire avant d'accepter."""
    absent = "" if _try_import_psutil() is not None else " — NON appliqué : psutil absent"
    return [
        f"CPU : {q.max_cpu_percent:g} % d'un cœur (arrêt au-delà du double, sur trois "
        f"mesures){absent}",
        f"Mémoire vive : {q.max_memory_mb:g} Mo{absent}",
        f"Durée : {q.max_runtime_s:g} s par appel (on_start, mission, on_stop)",
        f"Mémoire persistante : {MemoryService.MAX_CLES_PAR_APP} clés et "
        f"{MemoryService.MAX_OCTETS_PAR_APP // 1024} Kio au plus (fixé par l'OS)",
    ]


def decrire_prix(m: Any) -> str:
    """Ce que paiera le client, en clair."""
    if m.pricing_model == PricingModel.FREE:
        return "gratuite"
    modele = {PricingModel.ONE_TIME: "achat unique",
              PricingModel.SUBSCRIPTION: "abonnement, par période de 30 jours",
              PricingModel.PAY_PER_MISSION: "à chaque mission"}.get(m.pricing_model,
                                                                  m.pricing_model.name)
    return f"{m.store_price_eur:.2f} € ({modele})"


def empreinte_paquet(manifest: AppManifest, source: str) -> Optional[str]:
    """SHA-256 du contenu d'une app (manifeste, dans sa forme canonique
    manifeste_texte, et code), celle que garde le registre des apps : la
    même quel que soit le chemin d'installation (install, install_source).
    None, une fois la raison affichée, si ce contenu ne s'encode pas."""
    from axis_os.store.runtime.manifeste_json import manifeste_texte
    try:
        return hashlib.sha256(message_paquet(manifeste_texte(manifest), source)).hexdigest()
    except SignatureError as exc:
        print(f"  [ERREUR] {_affichable(exc, 300)}")
        return None


def decrire_signataire(details: Optional[DetailsConfirmation]) -> str:
    """La clé qui a signé l'app, telle que l'OS l'a vérifiée : son empreinte
    et le nom sous lequel elle est inscrite, jamais l'auteur déclaré."""
    if details is None or details.signataire is None:
        return "app NON signée (aucune identité vérifiée)"
    if details.fichier_cles is None:
        if details.nom_signataire:
            return f"{details.signataire} — {_affichable(details.nom_signataire)}"
        return f"{details.signataire} (aucun fichier de clés approuvées sur ce robot)"
    nom = _affichable(details.nom_signataire) if details.nom_signataire else "(sans nom)"
    return (f"{details.signataire} — « {nom} », fichier des clés approuvées "
            f"{_affichable(details.fichier_cles, 200)}")


def accepter_tout_SIMULATION(manifest: AppManifest,
                             permissions: List[Permission]) -> bool:
    """Accorde toutes les permissions DANGEROUS sans rien demander.

    Réservée à la simulation et aux tests. Son nom est choisi pour qu'on ne
    la passe pas par erreur sur un vrai robot ; en plus, PlayStoreRuntime
    refuse de la recevoir quand il pilote un robot réel connu : l'humanoïde
    Unitree G1 en backend DDS (Data Distribution Service, le bus de
    communication par lequel le vrai G1 reçoit ses commandes ; voir
    _robot_reel). Sur un vrai robot, utiliser confirmer_au_terminal ou un
    écran de confirmation."""
    print(f"  [SIMULATION] permissions DANGEROUS accordées SANS confirmation "
          f"humaine : {[p.name for p in permissions]}")
    return True


# Marque lue par PlayStoreRuntime : refusée sur un robot réel et en
# production. Un confirmateur maison qui accorde tout sans humain doit
# porter la même marque.
accepter_tout_SIMULATION.accorde_sans_humain = True


def _affichable(texte: Any, n: int = 80) -> str:
    """Rend inoffensif pour un terminal un texte venu de l'app.

    Pourquoi : un nom ou un auteur d'app peut contenir des caractères de
    contrôle (séquences d'échappement du terminal, sauts de ligne) qui
    fausseraient l'affichage de la question posée à l'opérateur. Ils sont
    remplacés par « ? », et la longueur est bornée à `n` caractères.
    validate() refuse déjà ces caractères ; ceci est une seconde
    barrière."""
    t = "".join(c if c.isprintable() else "?" for c in str(texte))
    return t if len(t) <= n else t[:n - 1] + "…"


def confirmer_au_terminal(manifest: AppManifest, permissions: List[Permission],
                          details: Optional[DetailsConfirmation] = None) -> bool:
    """Demande à l'opérateur, au terminal, d'accepter les permissions
    DANGEROUS, après lui avoir montré la clé qui a signé l'app et ses
    quotas.

    Seul « o » ou « oui » accorde ; tout le reste (Entrée seule, fin de
    saisie, erreur de lecture) refuse. Le nom, l'auteur et l'app_id viennent
    de l'app : ils sont affichés neutralisés par _affichable(), car la
    relecture de l'audit de sécurité du 2026-10-10 a montré qu'une
    séquence d'échappement placée dans ces champs pouvait masquer la vraie
    liste des permissions et afficher une fausse question. Pour la même
    raison, la question elle-même répète l'app_id et les permissions
    demandées."""
    app_id = _affichable(manifest.app_id)
    noms = ", ".join(p.name for p in permissions)
    if permissions:
        print(f"\n  L'app « {_affichable(manifest.name)} » ({app_id}, auteur déclaré : "
              f"{_affichable(manifest.author)}) demande des permissions DANGEREUSES :")
    else:
        print(f"\n  L'app « {_affichable(manifest.name)} » ({app_id}, auteur déclaré : "
              f"{_affichable(manifest.author)}) change de prix :")
    for p in permissions:
        print(f"    - {p.name}")
    print(f"  Prix       : {decrire_prix(manifest)}")
    if details is not None and details.prix_precedent:
        print(f"  ⚠ PRIX MODIFIÉ : la version installée coûtait {details.prix_precedent}")
    print(f"  Signée par : {decrire_signataire(details)}")
    if details is not None and details.liaison:
        print(f"  Registre   : {details.liaison}")
    print("  Ressources maximales :")
    for ligne in decrire_quotas(manifest.quota):
        print(f"    - {ligne}")
    question = (f"  Accorder {noms} à {app_id} ? [o/N] " if permissions
                else f"  Accepter le nouveau prix de {app_id} ? [o/N] ")
    try:
        rep = input(question)
    except (EOFError, OSError):
        return False
    return rep.strip().lower() in ("o", "oui")


confirmer_au_terminal.recoit_details = True


def _robot_reel(robot: Any) -> bool:
    """Vrai si le HAL pilote un robot réel connu.

    Le HAL (couche d'abstraction matérielle : l'interface commune par
    laquelle l'OS commande les moteurs et lit les capteurs) peut être
    enveloppé, par exemple par NormalizedRobotHAL, qui garde le HAL
    d'origine dans `_hal` ; on remonte au plus cinq niveaux. Aujourd'hui,
    seul le G1 en backend « dds » est reconnu comme robot réel.

    C'est un filet de sécurité pour accepter_tout_SIMULATION, pas une
    détection exhaustive : c'est d'abord le nom de cette fonction qui doit
    empêcher l'erreur."""
    vus = 0
    while robot is not None and vus < 5:
        if getattr(robot, "_backend", None) == "dds":
            return True
        robot, vus = getattr(robot, "_hal", None), vus + 1
    return False


@dataclass
class InstalledApp:
    """Une app installée sur ce robot, avec son manifeste validé, ses
    compteurs (missions, revenus) et son état de facturation."""
    app:           RobotApp
    manifest:      AppManifest
    install_time:  float = field(default_factory=time.monotonic)
    install_id:    str   = field(default_factory=lambda: str(uuid.uuid4())[:8])
    mission_count: int   = 0
    total_revenue: float = 0.0
    is_started:    bool  = False
    # État de facturation de l'app sur ce robot.
    #
    # Pourquoi ces deux champs : un audit de sécurité externe a montré que
    # _bill() facturait le prix complet à chaque mission, quel que soit le
    # modèle de prix (pricing_model). Une app ONE_TIME (achat unique)
    # était facturée à chaque mission au lieu d'une seule fois, et une app
    # SUBSCRIPTION (abonnement) facturait l'abonnement mensuel entier à
    # chaque mission au lieu d'une fois par période. Ces champs mémorisent
    # ce qui a déjà été facturé, pour que _bill() applique la bonne règle
    # à chaque modèle (détail dans sa docstring).
    #
    # Attention : sans memory_service, cet état n'existe qu'en mémoire et
    # se perd à chaque redémarrage du processus. Un second audit externe
    # l'a relevé : un redémarrage refacture alors une app ONE_TIME comme
    # neuve et remet à zéro la fenêtre SUBSCRIPTION de 30 jours. La
    # persistance sur disque a été ajoutée depuis, en option, par le
    # paramètre memory_service de PlayStoreRuntime (voir __init__). Sans
    # lui, la limite demeure : à traiter avant tout déploiement où un
    # redémarrage de PlayStoreRuntime est un événement réaliste en
    # production.
    #
    # last_subscription_bill_time_s utilise time.time(), l'horloge murale
    # (la date et l'heure réelles), et non time.monotonic(). Correction
    # faite après le second audit externe, qui a relevé que monotonic() ne
    # représente pas une date : sa valeur n'a de sens que comme différence
    # entre deux appels dans le même processus, jamais pour dire « il y a
    # 30 jours », et son point de départ est arbitraire et change à chaque
    # redémarrage. Ce défaut aggravait concrètement, et pas seulement en
    # théorie, le problème de persistance décrit ci-dessus : une valeur
    # monotonic() enregistrée puis relue après un redémarrage ne veut plus
    # rien dire.
    one_time_billed:              bool            = False
    last_subscription_bill_time_s: Optional[float] = None
    # Comment le processus de l'app retrouve son code (voir
    # processus_app.spec_depuis_instance), fixé à l'installation. None si
    # l'app ne peut pas être rechargée ; spec_erreur dit alors pourquoi, et
    # chacun de ses hooks échoue avec SandboxErrorKind.SANDBOX_ERROR.
    spec:       Optional[Dict[str, Any]] = None
    spec_erreur: Optional[str]           = None

    @property
    def commission_due(self) -> float:
        return round(self.total_revenue * AppManifest.COMMISSION_RATE, 4)


@dataclass
class BillingReport:
    """Reçu d'une facturation : montant payé par le client, commission de
    la plateforme (30 %) et part du développeur (70 %)."""
    mission_id:       str
    app_id:           str
    pricing_model:    PricingModel
    gross_amount_eur: float
    commission_eur:   float
    developer_eur:    float
    timestamp:        float = field(default_factory=time.time)

    def __str__(self) -> str:
        return (f"[BILLING] {self.app_id} | mission={self.mission_id} | "
                f"total={self.gross_amount_eur:.2f}€ | "
                f"commission={self.commission_eur:.2f}€ (30%) | "
                f"dev={self.developer_eur:.2f}€ (70%)")


class PlayStoreRuntime:
    """
    Moteur d'exécution du Play Store sur un robot : installe les apps, les
    démarre, exécute leurs missions dans la sandbox, les arrête, les met à
    jour et tient la facturation.

    Usage :
        runtime = PlayStoreRuntime(norm_robot, watchdog,
                                   confirmer_permissions=confirmer_au_terminal)
        runtime.install(PackageSortingApp())
        runtime.start_app("com.acme.package_sorting")
        result = runtime.run_mission("com.acme.package_sorting", params={})
    """

    def __init__(
        self,
        robot: NormalizedRobotHAL,
        watchdog: WatchdogSystem,
        require_signed_apps: Optional[bool] = None,
        platform_public_key: Optional[str] = None,
        memory_service: Optional[MemoryService] = None,
        perception_service=None,
        locomotion_service=None,
        speech_service=None,
        clock=None,
        confirmer_permissions: Optional[ConfirmationPermissions] = None,
        *,
        profil: Profil,
        cles_approuvees: Optional[Union[str, os.PathLike]] = None,
        registre_apps: Optional[Union[str, os.PathLike]] = None,
        navigation_service=None,
        manipulation_service=None,
        human_interface=None,
        speech_synthesizer=None,
    ):
        """
        `profil` (obligatoire, jamais deviné) : SIMULATION, DEVELOPPEMENT ou
        PRODUCTION (voir axis_os/store/runtime/profil.py). En PRODUCTION :
        signature obligatoire (require_signed_apps ne peut pas valoir
        False), clés de développeur lues dans `cles_approuvees` (fichier
        local, obligatoire), accepter_tout_SIMULATION refusé, quotas CPU et
        mémoire exigés (psutil), et installation depuis la source signée
        seulement (install_source) : le code d'une app n'est jamais exécuté
        dans le processus de l'OS. SIMULATION est refusé sur un robot réel.
        En SIMULATION et en DÉVELOPPEMENT, les facilités historiques restent
        possibles, mais chacune est signalée dans le journal ([PROFIL …]).

        `cles_approuvees` : chemin du fichier des clés publiques de
        développeur approuvées (format dans profil.py). Fourni hors
        production, il est appliqué de même.

        `registre_apps` : chemin du registre persistant des apps (format
        dans profil.py), obligatoire en PRODUCTION : chaque app_id y est lié
        à la clé qui l'a installé la première fois, avec la version la plus
        récente installée. Tout chemin d'installation le consulte et le
        tient à jour (install_source, update_source, et hors production
        install et update_app) ; une app non signée ne peut pas prendre un
        app_id déjà lié.

        `robot` : le HAL du robot, vu à travers NormalizedRobotHAL, la
        façade qui présente aux apps les positions et vitesses dans une
        convention commune plutôt que celle du driver brut.
        `watchdog` : le chien de garde du robot, consulté avant chaque
        mission pour savoir si un e-stop a été déclenché, et transmis au
        RobotContext de chaque app.

        `confirmer_permissions` : fonction (manifeste, permissions
        DANGEROUS) -> bool. install() l'appelle (donc aussi update_app(),
        qui passe par install()) pour toute app qui déclare au moins une
        permission de niveau ProtectionLevel.DANGEROUS ; seule la réponse
        True accorde. None par défaut : aucune permission DANGEROUS n'est
        accordée et l'app est refusée. En cas de doute, le système refuse
        (comportement dit fail-safe) ; voir accepter_tout_SIMULATION et
        confirmer_au_terminal plus haut. Passer accepter_tout_SIMULATION
        avec un robot réel lève ValueError. Un confirmateur qui porte
        `recoit_details = True` reçoit en troisième argument un
        DetailsConfirmation (clé signataire vérifiée) ; il doit montrer à
        l'humain cette clé et manifest.quota. En PRODUCTION, c'est exigé.

        `require_signed_apps` : si True, install() refuse toute app dont
        code_signature et developer_public_key ne sont pas renseignés.
        False par défaut, pour rester compatible avec les apps existantes
        non signées. Pourquoi ce paramètre : un audit de sécurité a montré
        que la signature était jusque-là facultative, au choix de l'app
        (les deux champs valent None par défaut) ; une app malveillante
        n'avait qu'à omettre le champ pour s'installer sans aucune
        vérification. Le CHANGELOG présentait la signature comme réglant
        le fait que « rien n'empêchait un fichier .py quelconque de se
        faire passer pour une app légitime » : ce n'est vrai qu'avec ce
        paramètre à True. À activer explicitement avant d'ouvrir un vrai
        Play Store à des développeurs externes non vérifiés.

        `platform_public_key` : clé publique Ed25519 (un algorithme de
        signature numérique ; même format que
        AppManifest.developer_public_key) qui distingue une app système
        d'une app tierce, voir ProtectionLevel.SIGNATURE dans
        axis_os/store/sdk/manifest.py. Une app dont developer_public_key
        est exactement cette clé, et dont la signature est valide, est
        traitée comme signée par la plateforme : elle peut déclarer des
        permissions de niveau SIGNATURE (par exemple ACTUATORS_RAW, les
        commandes articulaires brutes). None par défaut (compatible avec
        l'existant) : sans clé plateforme configurée, aucune app, même
        correctement signée par un vrai développeur, n'est traitée comme
        système, et toute permission SIGNATURE est refusée à
        l'installation. Là encore, le défaut est le refus (fail-safe),
        jamais l'accord (fail-open).

        `memory_service` : service de mémoire persistante (voir
        axis_os/services/memory/service.py), qui doit être déjà démarré
        par MemoryService.start(). S'il est fourni, l'état de facturation
        (one_time_billed, last_subscription_bill_time_s) y est lu à
        l'installation et réécrit à chaque changement. Cela comble le trou
        relevé par deux audits externes successifs : cet état n'existait
        qu'en mémoire et se perdait à chaque redémarrage (une app ONE_TIME
        refacturée comme neuve, une fenêtre SUBSCRIPTION de 30 jours
        remise à zéro). None par défaut (compatible avec l'existant) :
        comportement strictement inchangé, état en mémoire seulement,
        comme avant ce paramètre. Essais : test/test_billing_persistence.py.

        `perception_service`, `locomotion_service`, `speech_service` :
        services système (vision, marche, parole) transmis à chaque app
        par son RobotContext ; voir _make_ctx(). `clock` : horloge du
        robot, voir le commentaire dans le corps de cette méthode.

        `navigation_service`, `manipulation_service` : services servis par
        ctx.go_to() et ctx.pick() / place() ; None : l'app reçoit
        CAPABILITY_MISSING. Leur tick() reste l'affaire de la boucle du
        robot.

        `human_interface` : IHumanInterface qui répond à
        ctx.ask_confirmation() ; None : aucun humain (refus).

        `speech_synthesizer` : ISpeechSynthesizer de ctx.speak() ; None :
        pas de synthèse vocale (CAPABILITY_MISSING).
        """
        if not isinstance(profil, Profil):
            raise TypeError(
                "profil doit être Profil.SIMULATION, Profil.DEVELOPPEMENT ou "
                "Profil.PRODUCTION, donné explicitement (il n'est jamais deviné).")
        reel = _robot_reel(robot)
        if profil is Profil.SIMULATION and reel:
            raise ValueError(
                "Profil SIMULATION refusé : ce runtime pilote un robot RÉEL. "
                "Choisir DEVELOPPEMENT (banc d'essai) ou PRODUCTION.")
        sans_humain = getattr(confirmer_permissions, "accorde_sans_humain", False) is True
        if sans_humain and reel:
            raise ValueError(
                "accepter_tout_SIMULATION refusé : ce runtime pilote un robot RÉEL. "
                "Les permissions DANGEROUS doivent y être confirmées par un humain "
                "(confirmer_au_terminal, ou un écran de confirmation).")
        if profil is Profil.PRODUCTION:
            if sans_humain:
                raise ValueError(
                    "Profil PRODUCTION : accepter_tout_SIMULATION refusé, les "
                    "permissions DANGEROUS y sont toujours confirmées par un humain.")
            if confirmer_permissions is not None and \
                    getattr(confirmer_permissions, "recoit_details", False) is not True:
                raise ValueError(
                    "Profil PRODUCTION : le confirmateur doit montrer à l'humain la clé "
                    "signataire et les quotas (attribut recoit_details = True, voir "
                    "DetailsConfirmation et confirmer_au_terminal).")
            if require_signed_apps is False:
                raise ValueError(
                    "Profil PRODUCTION : la signature des apps est obligatoire "
                    "(require_signed_apps=False refusé).")
            require_signed_apps = True
            if cles_approuvees is None:
                raise ValueError(
                    "Profil PRODUCTION : fichier des clés de développeur approuvées "
                    "requis (paramètre cles_approuvees, format dans profil.py).")
            if _try_import_psutil() is None:
                raise RuntimeError(
                    "Profil PRODUCTION : psutil est requis pour appliquer les quotas "
                    "CPU et mémoire des apps.")
            if not rendre_os_non_inspectable():
                raise RuntimeError(
                    "Profil PRODUCTION : impossible d'interdire l'inspection du "
                    "processus de l'OS (prctl PR_SET_DUMPABLE) ; une app pourrait lire "
                    "son environnement ou sa mémoire.")
        self._profil = profil
        self._cles_approuvees: Optional[Dict[str, str]] = (
            None if cles_approuvees is None else lire_cles_approuvees(cles_approuvees))
        self._fichier_cles = None if cles_approuvees is None else os.fspath(cles_approuvees)
        if profil is Profil.PRODUCTION and registre_apps is None:
            raise ValueError(
                "Profil PRODUCTION : registre des apps requis (paramètre registre_apps, "
                "format dans profil.py ; le créer une fois par creer_registre_apps).")
        self._chemin_registre = None if registre_apps is None else os.fspath(registre_apps)
        self._registre: Optional[Dict[str, Dict[str, str]]] = (
            None if registre_apps is None else lire_registre_apps(registre_apps))
        self._confirmer_permissions = confirmer_permissions
        self._robot   = robot
        self._watchdog = watchdog
        self._apps:    Dict[str, InstalledApp] = {}
        self._billing: List[BillingReport]     = []
        self._id = str(uuid.uuid4())[:8]
        self._require_signed_apps = bool(require_signed_apps)
        if profil is not Profil.PRODUCTION:
            # Chaque facilité est dite au démarrage : un choix silencieux
            # s'oublie facilement, et finit dans une configuration de
            # production.
            facilites = []
            if not self._require_signed_apps:
                facilites.append("apps non signées acceptées")
            if self._cles_approuvees is None:
                facilites.append("toute clé de développeur acceptée")
            if sans_humain:
                facilites.append("permissions DANGEROUS accordées sans humain")
            facilites.append("installation d'objets app chargés dans l'OS")
            print(f"  [PROFIL {profil.name}] ne convient pas à un vrai robot en "
                  f"service : {', '.join(facilites)}.")
        if not self._require_signed_apps:
            # Contexte complet dans la docstring de __init__ : sans
            # require_signed_apps=True, n'importe quel fichier .py dont une
            # classe hérite de RobotApp s'installe sans vérification
            # d'identité.
            print(
                "  [SECURITY] PlayStoreRuntime démarré avec "
                "require_signed_apps=False — toute app non signée "
                "s'installe sans vérification d'identité. Adapté au "
                "développement, à activer explicitement avant "
                "d'ouvrir ce Play Store à des développeurs externes."
            )
        self._platform_public_key = platform_public_key
        self._memory_service = memory_service
        self._perception_service = perception_service
        self._locomotion_service = locomotion_service
        self._speech_service = speech_service
        # Services de haut niveau, servis par ctx.go_to(), pick(), place().
        # Ils restent tickés par la boucle du robot, jamais par une app.
        self._navigation_service = navigation_service
        self._manipulation_service = manipulation_service
        # Interface humaine (IHumanInterface) : celle qui répond à
        # ctx.ask_confirmation(). None : aucun humain, la question est
        # refusée (HUMAN_UNAVAILABLE), jamais tenue pour acceptée.
        self._human_interface = human_interface
        self._speech_synthesizer = speech_synthesizer
        # Réglages choisis par l'utilisateur, par app (set_setting).
        self._reglages: Dict[str, Dict[str, Any]] = {}
        # Contexte du hook en cours, par app (pour request_stop), et
        # dernière progression rapportée (get_progress).
        # Plusieurs hooks d'une même app peuvent tourner à la fois (stop_app
        # pendant une mission) : une liste par app.
        self._ctx_courants: Dict[str, List[RobotContext]] = {}
        self._progressions: Dict[str, Any] = {}
        # Horloge du robot : simulée en simulation, réelle sur le robot.
        # Elle est transmise à la sandbox (pour mesurer les quotas) et aux
        # apps (ctx.now / ctx.sleep). C'est l'équivalent de use_sim_time
        # dans ROS (Robot Operating System, l'intergiciel robotique de
        # référence), qui fait suivre aux programmes l'horloge du
        # simulateur.
        self._clock = clock or time.monotonic
        # Jeton de session. Pourquoi : sans lui, une app (ou tout code
        # tiers) pourrait contourner install() en construisant son propre
        # RobotContext avec un manifeste arbitraire. Avec lui, seule cette
        # instance de PlayStoreRuntime peut construire un RobotContext
        # utilisable. Voir _RuntimeSessionToken et UnauthorizedContextError
        # dans axis_os/store/sdk/app_base.py, et
        # test/test_context_lockdown.py.
        self._session_token = _RuntimeSessionToken()
        # Verrou global de mission. Pourquoi : un audit a montré que rien
        # n'empêchait deux appels concurrents à run_mission() (deux apps
        # différentes, ou la même app deux fois) d'envoyer en même temps
        # des commandes contradictoires aux mêmes actionneurs physiques.
        # Règle : un seul robot, une seule mission active à la fois,
        # quelle que soit l'app. Ce n'est pas une file d'attente : un appel
        # concurrent est refusé immédiatement, et non mis en attente en
        # silence, pour rester prévisible et éviter qu'un appelant croie
        # sa mission en cours alors qu'elle attend indéfiniment derrière
        # une autre.
        self._mission_lock = threading.Lock()
        # Une installation ou mise à jour depuis la source à la fois : sans
        # lui, deux installations du même app_id par deux clés passeraient
        # toutes deux le contrôle du registre.
        self._verrou_installation = threading.Lock()
        # app_id en cours de mise à jour (update_source, update_app) : son
        # démarrage est refusé tant que la mise à jour n'est pas finie.
        self._maj_en_cours: Optional[str] = None
        print(f"\n[PlayStore] Runtime démarré | id={self._id} | robot={robot.robot_id}")

    # ── Installation ──────────────────────────────────────────────

    def install(self, app: RobotApp) -> bool:
        """
        Installe une app sur ce robot. Renvoie True si elle est installée,
        False (raison affichée) dès qu'un contrôle échoue.

        Ordre des contrôles : validité du manifeste, compatibilité du SDK,
        signature du code, registre des apps s'il y en a un, plafonds des
        quotas, permissions réservées au système (SIGNATURE), capacités
        fonctionnelles, matériel, puis, en dernier, confirmation humaine des
        permissions DANGEROUS. Une seule installation à la fois.
        """
        if not self._verrou_installation.acquire(blocking=False):
            print("  [ERREUR] Une autre installation est en cours sur ce robot : refusé.")
            return False
        try:
            return self._installer_objet(app)
        finally:
            self._verrou_installation.release()

    def _installer_objet(self, app: RobotApp) -> bool:
        """Corps d'install(), sous le verrou d'installation."""
        if self._profil is Profil.PRODUCTION:
            # L'objet app a déjà été créé dans ce processus : son code (son
            # module, son constructeur, et manifest() juste après) y
            # tourne avec tous les droits de l'OS.
            print("  [ERREUR] Profil PRODUCTION : une app s'installe depuis sa source "
                  "signée (install_source), jamais depuis un objet déjà chargé dans "
                  "le processus de l'OS.")
            return False

        # On travaille sur une copie du manifeste, validée puis conservée :
        # RobotContext consultera celle-ci, jamais l'objet fourni par
        # l'app, que l'app pourrait modifier après la confirmation des
        # permissions (relevé à la relecture de l'audit de sécurité du
        # 2026-10-10).
        manifest = copy.deepcopy(app.manifest())
        app_id   = manifest.app_id

        print(f"\n[PlayStore] Installation '{manifest.name}' v{manifest.version}")
        print(f"  Auteur    : {manifest.author}")
        print(f"  Catégorie : {manifest.category.value}")

        errors = manifest.validate()
        if errors:
            # Un message de validate() peut reprendre un champ de l'app.
            for e in errors: print(f"  [ERREUR] {_affichable(e, 300)}")
            return False

        # Compatibilité du SDK, dont la version est celle d'AXIS-OS (voir
        # axis_os/__init__.py ; versions au format SemVer, le
        # versionnement sémantique majeur.mineur.correctif). Vérifiée
        # avant le reste : une app écrite pour une version incompatible
        # peut attendre un autre format pour HardwareRequirements ou
        # Capabilities, contrôlés plus loin ; mieux vaut échouer ici avec
        # un message clair qu'avec une erreur confuse ensuite.
        if not self._sdk_compatible(manifest):
            return False

        # Signature du code (voir axis_os/store/sdk/signing.py). Contrôlée
        # après le SDK (inutile d'aller plus loin si le SDK est déjà
        # incompatible) et avant le matériel (une signature invalide est
        # une raison de refus plus fondamentale qu'une capacité
        # manquante).
        #
        # Un audit de sécurité a montré que cette vérification était
        # facultative : elle n'avait lieu que si l'app avait choisi de
        # renseigner les deux champs. self._require_signed_apps permet de
        # la rendre obligatoire (contexte complet dans __init__).
        if self._require_signed_apps and manifest.code_signature is None:
            print(
                "  [ERREUR] Ce Play Store exige des apps signées "
                "(require_signed_apps=True) — code_signature et "
                "developer_public_key doivent être renseignés."
            )
            return False

        source_verifiee: Optional[str] = None
        if manifest.code_signature is not None:
            from axis_os.store.sdk.signing import verify_app_signature, SignatureError
            import inspect
            try:
                # La signature porte sur le fichier source entier (le
                # module), pas seulement sur le texte de la classe. Un
                # audit de sécurité a jugé insuffisant
                # inspect.getsource(type(app)), qui ne couvre ni le reste
                # du module, ni ses imports, ni ses classes de base : du
                # code placé au niveau du module (une fonction, une
                # variable globale exécutée à l'import) pouvait être
                # modifié après signature sans l'invalider, puisqu'il ne
                # faisait pas partie du texte signé et vérifié.
                source_file = inspect.getsourcefile(type(app))
                if source_file is None:
                    raise OSError("aucun fichier source associé à cette classe")
                with open(source_file, "r", encoding="utf-8") as f:
                    source = f.read()
            except (OSError, TypeError) as exc:
                print(
                    f"  [ERREUR] Impossible de récupérer le code source de "
                    f"l'app pour vérifier sa signature : {exc}"
                )
                return False
            try:
                valid = verify_app_signature(
                    source, manifest.code_signature, manifest.developer_public_key
                )
            except SignatureError as exc:
                print(f"  [ERREUR] Signature ou clé malformée : {exc}")
                return False
            if not valid:
                print(
                    "  [ERREUR] Signature invalide — le code de l'app ne "
                    "correspond pas à sa signature déclarée (code modifié "
                    "après signature, ou mauvaise clé)."
                )
                return False
            # C'est ce texte précis, et non le fichier relu plus tard sur le
            # disque, que le processus de l'app exécutera.
            source_verifiee = source
            if not self._cle_approuvee(manifest.developer_public_key):
                return False

        # Comment le processus de l'app retrouvera son code. Une app qui ne
        # peut pas être rechargée (classe définie dans une fonction)
        # s'installe quand même, comme avant, mais chacun de ses hooks
        # échouera avec un message clair.
        try:
            spec, spec_erreur = spec_depuis_instance(app, source_verifiee), None
        except (AppNonRechargeable, OSError, UnicodeDecodeError) as exc:
            spec, spec_erreur = None, str(exc)
            print(f"  [ATTENTION] {spec_erreur}")
        # Avec un registre des apps (obligatoire en production, facultatif
        # ici), mêmes règles que install_source() : un app_id lié à une clé
        # ne s'installe qu'avec elle, jamais en version antérieure.
        avant_enregistrement = None
        if self._registre is not None:
            try:
                version = SemVer.parse(manifest.version)
            except ValueError as exc:
                print(f"  [ERREUR] {_affichable(exc, 300)}")
                return False
            if manifest.code_signature is None:
                if app_id in self._registre:
                    print(f"  [ERREUR] '{_affichable(app_id)}' est lié à une clé sur ce robot : "
                          f"une app non signée ne peut pas prendre cet app_id.")
                    return False
            else:
                empreinte = empreinte_paquet(manifest, source_verifiee)
                if empreinte is None or not self._verifier_registre(manifest, version, empreinte):
                    return False
                avant_enregistrement = lambda: self._lier(manifest, version, empreinte)  # noqa: E731
        return self._terminer_installation(manifest, app, spec, spec_erreur,
                                           avant_enregistrement=avant_enregistrement)

    def _terminer_installation(self, manifest: AppManifest, app: Optional[RobotApp],
                               spec: Optional[Dict[str, Any]],
                               spec_erreur: Optional[str],
                               avant_enregistrement: Optional[Callable[[], bool]] = None
                               ) -> bool:
        """Contrôles communs à install(), install_source() et
        update_source(), une fois le manifeste obtenu et la signature
        vérifiée : plafonds des quotas, permissions SIGNATURE, capacités,
        matériel, confirmation des permissions DANGEROUS, puis
        `avant_enregistrement` (l'écriture du registre des apps) et
        l'enregistrement."""
        app_id = manifest.app_id
        # Plafonds de l'OS (profil.PLAFONDS_QUOTAS), vérifiés avant tout le
        # reste, et avant l'humain : un quota trop grand n'est jamais ramené
        # en silence, l'installation est refusée. validate() a déjà vérifié
        # que chaque quota est un nombre fini strictement positif.
        plafond = PLAFONDS_QUOTAS[self._profil]
        depasses = [f.name for f in fields(plafond)
                    if getattr(manifest.quota, f.name) > getattr(plafond, f.name)]
        for nom in depasses:
            print(f"  [ERREUR] Quota {nom} = {getattr(manifest.quota, nom):g} {_UNITES[nom]} : "
                  f"au-delà du plafond du profil {self._profil.name} "
                  f"({getattr(plafond, nom):g} {_UNITES[nom]}). Installation refusée ; l'OS "
                  f"ne réduit jamais un quota déclaré : corriger le manifeste.")
        if depasses:
            return False
        # Permissions de niveau ProtectionLevel.SIGNATURE, réservées aux
        # apps système (par exemple ACTUATORS_RAW, les commandes
        # articulaires brutes, ou FLEET_BROADCAST, l'envoi d'un message à
        # toute une flotte de robots ; EMERGENCY_STOP, elle, est
        # délibérément DANGEROUS, voir manifest.py). Pourquoi (voir
        # docs/architecture_services.md, §6.3) : elles ne doivent être
        # accordées qu'à une app dont la signature vérifiée correspond
        # exactement à self._platform_public_key, jamais à n'importe
        # quelle app correctement signée par un développeur tiers. Si
        # aucune clé plateforme n'est configurée
        # (self._platform_public_key is None), toute permission SIGNATURE
        # est refusée : le défaut est le refus, jamais l'accord.
        is_platform_app = (
            self._platform_public_key is not None
            and manifest.code_signature is not None
            and manifest.developer_public_key == self._platform_public_key
        )
        for perm in manifest.permissions:
            if get_protection_level(perm) == ProtectionLevel.SIGNATURE and not is_platform_app:
                print(
                    f"  [ERREUR] Permission '{perm.name}' réservée aux apps "
                    f"système (ProtectionLevel.SIGNATURE) — cette app n'est "
                    f"pas signée par la clé plateforme."
                )
                return False

        # Capacités fonctionnelles (Capability : ce que le robot sait faire,
        # comme marcher ou parler), à ne pas confondre avec RobotCapability
        # (le matériel présent) : voir la docstring de Capability dans
        # manifest.py. Ce contrôle est bloquant, contrairement à
        # hw.optional plus bas : une app qui déclare avoir besoin d'une
        # Capability en a besoin, il n'existe pas ici de notion
        # « optionnelle » comme dans HardwareRequirements. Une app qui veut
        # fonctionner en mode dégradé sans une capacité ne la déclare
        # simplement pas dans capabilities_required.
        if manifest.capabilities_required:
            available = self._available_capabilities()
            missing_caps = [c for c in manifest.capabilities_required if c not in available]
            if missing_caps:
                names = sorted(c.name for c in missing_caps)
                print(f"  [ERREUR] Capacité(s) fonctionnelle(s) requise(s) "
                      f"absente(s) sur ce robot : {names}")
                return False

        # Matériel : nombre minimal de degrés de liberté (DoF, en pratique
        # le nombre d'articulations motorisées) et modèles de robot
        # compatibles.
        hw = manifest.hardware
        robot_info = self._robot.get_robot_info()
        robot_dof  = int(robot_info.get("dof", 0))
        if hw.min_dof > 0 and robot_dof < hw.min_dof:
            print(f"  [ERREUR] Requiert {hw.min_dof} DoF, robot en a {robot_dof}")
            return False
        if hw.compatible_models:
            model = robot_info.get("model", "")
            if model not in hw.compatible_models:
                print(f"  [ERREUR] Modèle '{model}' non supporté. Requis: {hw.compatible_models}")
                return False

        # Capacités matérielles : une capacité de hw.required qui manque
        # bloque l'installation ; une capacité de hw.optional qui manque
        # donne seulement un avertissement. Voir
        # HardwareRequirements.required et .optional dans manifest.py.
        caps = self._robot.get_capabilities()
        if hw.required:
            missing = caps.missing_from(hw.required)
            if missing:
                names = sorted(c.name for c in missing)
                print(f"  [ERREUR] Capacité(s) requise(s) manquante(s) : {names}")
                return False
        if hw.optional:
            missing_optional = caps.missing_from(hw.optional)
            if missing_optional:
                names = sorted(c.name for c in missing_optional)
                print(f"  [ATTENTION] Capacité(s) optionnelle(s) absente(s) — "
                      f"l'app fonctionnera en mode dégradé : {names}")

        # Ce que l'humain doit savoir, dit aussi dans le journal pour une app
        # qui ne demande aucune permission DANGEROUS : qui a signé (la clé
        # vérifiée, pas l'auteur déclaré) et les ressources accordées.
        details = self._details_confirmation(manifest)
        print(f"  Signée par : {decrire_signataire(details)}")
        print(f"  Quotas    : {' ; '.join(decrire_quotas(manifest.quota))}")
        print(f"  Prix      : {decrire_prix(manifest)}")
        # Une nouvelle version (mise à jour, ou réinstallation par-dessus)
        # qui change le modèle de prix ou augmente le prix doit être acceptée
        # par l'humain, même sans permission DANGEROUS.
        prix_change = details.prix_precedent is not None

        # Permissions DANGEROUS : confirmation humaine explicite, demandée en
        # dernier pour ne pas faire accepter à un humain une app qui serait
        # refusée de toute façon (capacités, matériel). Voir l'en-tête de
        # section au-dessus de accepter_tout_SIMULATION.
        dangereuses = list(dict.fromkeys(
            p for p in manifest.permissions
            if get_protection_level(p) == ProtectionLevel.DANGEROUS))
        if dangereuses or prix_change:
            noms = [p.name for p in dangereuses] or ["(nouveau prix)"]
            if self._confirmer_permissions is None:
                print(f"  [ERREUR] Permission(s) DANGEROUS ou nouveau prix {noms} : aucune "
                      f"confirmation configurée sur ce runtime (confirmer_permissions) — "
                      f"refus par défaut.")
                return False
            try:
                if getattr(self._confirmer_permissions, "recoit_details", False) is True:
                    accord = self._confirmer_permissions(manifest, list(dangereuses), details)
                else:
                    accord = self._confirmer_permissions(manifest, list(dangereuses))
            except Exception as exc:          # noqa: BLE001 — refus, jamais accord
                print(f"  [ERREUR] Confirmation des permissions {noms} impossible "
                      f"({exc!r:.120}) — refus.")
                return False
            if accord is not True:
                # « is True » plutôt qu'un simple test de vérité : une
                # réponse floue (chaîne, 1, None) n'est pas un accord.
                print(f"  [ERREUR] Permission(s) DANGEROUS {noms} non accordée(s) — "
                      f"installation refusée.")
                return False
            print(f"  Permissions DANGEROUS accordées explicitement : {noms}")

        # Récapitulatif des permissions déclarées.
        print(f"  Permissions : {[p.name for p in manifest.permissions]}")

        # Prix payé par le client et partage : part du développeur,
        # commission de 30 % pour la plateforme.
        if manifest.pricing_model != PricingModel.FREE:
            print(f"  Prix client : {manifest.store_price_eur:.2f}€ "
                  f"| Dev : {manifest.developer_revenue_eur:.2f}€ "
                  f"| Commission : {manifest.platform_commission_eur:.2f}€ (30%)")

        # État de facturation repris depuis MemoryService s'il est fourni
        # (contexte complet : paramètre memory_service de __init__). Sans
        # lui, comportement strictement inchangé par rapport à l'époque
        # où ce paramètre n'existait pas : one_time_billed=False et
        # last_subscription_bill_time_s=None. Une app ONE_TIME ou
        # SUBSCRIPTION redémarrée sans memory_service est donc facturée
        # comme neuve : c'est le comportement documenté et assumé quand
        # la persistance n'est pas branchée.
        one_time_billed = False
        last_subscription_bill_time_s = None
        if self._memory_service is not None:
            one_time_billed = self._read_billing_flag(app_id, "one_time_billed", False)
            last_subscription_bill_time_s = self._read_billing_flag(
                app_id, "last_subscription_bill_time_s", None)

        # Le vocabulaire vocal d'une version précédente ne survit pas à une
        # nouvelle installation sous le même app_id : son on_start le redéclare.
        registre_vocal = getattr(self._speech_service, "registry", None)
        # Dernier contrôle, après l'humain : l'écriture du registre des apps.
        # Si elle échoue, rien n'est installé (une liaison perdue au
        # redémarrage laisserait une autre clé reprendre l'app_id).
        if avant_enregistrement is not None and not avant_enregistrement():
            return False
        if registre_vocal is not None:
            registre_vocal.retirer(app_id)
        self._apps[app_id] = InstalledApp(
            app=app, manifest=manifest, spec=spec, spec_erreur=spec_erreur,
            one_time_billed=one_time_billed,
            last_subscription_bill_time_s=last_subscription_bill_time_s,
        )
        print(f"  ✓ '{manifest.name}' installée.")
        if manifest.code_signature is None:
            print(f"  [PROFIL {self._profil.name}] app non signée installée : "
                  f"{_affichable(app_id)}")
        return True

    def _details_confirmation(self, manifest: AppManifest) -> DetailsConfirmation:
        """Signataire vérifié, liaison et prix d'un manifeste (voir
        DetailsConfirmation)."""
        details = self._details_signataire(manifest)
        installee = self._apps.get(manifest.app_id)
        precedent = None
        if installee is not None:
            ancien = installee.manifest
            if ancien.pricing_model != manifest.pricing_model or \
                    manifest.store_price_eur > ancien.store_price_eur:
                precedent = decrire_prix(ancien)
        return replace(details, prix=decrire_prix(manifest), prix_precedent=precedent)

    def _details_signataire(self, manifest: AppManifest) -> DetailsConfirmation:
        cle = manifest.developer_public_key if manifest.code_signature is not None else None
        liaison = None
        if self._registre is not None and cle is not None:
            lien = self._registre.get(manifest.app_id)
            liaison = ("première installation de cet app_id sur ce robot : il sera lié à "
                       "cette clé" if lien is None else
                       f"app_id déjà lié à cette clé, version la plus récente installée "
                       f"{lien['version']}")
        if cle is None:
            return DetailsConfirmation(self._profil.name, None, None, self._fichier_cles, liaison)
        empreinte = public_key_fingerprint(cle)
        if self._cles_approuvees is not None and cle in self._cles_approuvees:
            return DetailsConfirmation(self._profil.name, empreinte,
                                       self._cles_approuvees[cle], self._fichier_cles, liaison)
        if cle == self._platform_public_key:
            return DetailsConfirmation(self._profil.name, empreinte, "clé plateforme", None,
                                       liaison)
        return DetailsConfirmation(self._profil.name, empreinte, None, self._fichier_cles,
                                   liaison)

    def _verifier_registre(self, manifest: AppManifest, version: SemVer,
                           empreinte: Optional[str] = None) -> bool:
        """Règles du registre des apps, avant l'humain : un app_id lié à une
        clé ne s'installe qu'avec elle, et jamais dans une version antérieure
        à la plus récente déjà installée. Une version égale n'est acceptée
        que pour le même contenu signé (`empreinte`, SHA-256 de ce qui est
        signé) : c'est une réinstallation, après un redémarrage par exemple ;
        un autre contenu exige un numéro plus grand."""
        if self._registre is None:
            return True
        lien = self._registre.get(manifest.app_id)
        if lien is None:
            return True
        app_id = _affichable(manifest.app_id)
        if lien["cle"] != manifest.developer_public_key:
            print(f"  [ERREUR] '{app_id}' est lié sur ce robot à la clé "
                  f"{public_key_fingerprint(lien['cle'])}, qui l'a installé la première fois ; "
                  f"ce paquet est signé par {public_key_fingerprint(manifest.developer_public_key)}. "
                  f"Une autre clé ne peut pas reprendre un app_id : refusé.")
            return False
        if version < SemVer.parse(lien["version"]):
            print(f"  [ERREUR] '{app_id}' v{version} : antérieure à la version la plus récente "
                  f"déjà installée sur ce robot (v{lien['version']}) : refusé.")
            return False
        if (version == SemVer.parse(lien["version"]) and empreinte is not None
                and lien.get("empreinte") not in (None, empreinte)):
            print(f"  [ERREUR] '{app_id}' v{version} : ce numéro a déjà été installé sur ce robot "
                  f"avec un AUTRE contenu signé ; un nouveau contenu exige un numéro de version "
                  f"plus grand : refusé.")
            return False
        return True

    def _lier(self, manifest: AppManifest, version: SemVer,
              empreinte: Optional[str] = None) -> bool:
        """Inscrit au registre des apps (sur disque) la liaison app_id -> clé et
        la version la plus récente installée. False si l'écriture échoue."""
        if self._registre is None:
            return True
        ancien = self._registre.get(manifest.app_id)
        if ancien is not None and ancien["cle"] != manifest.developer_public_key:
            print(f"  [ERREUR] '{_affichable(manifest.app_id)}' a été lié à une autre clé "
                  f"pendant l'installation : refusé.")
            return False
        lien = {"cle": manifest.developer_public_key, "version": str(version)}
        if empreinte is not None:
            lien["empreinte"] = empreinte
        if ancien is not None:
            inscrite = SemVer.parse(ancien["version"])
            # La plus récente reste celle déjà inscrite, avec son contenu ;
            # une entrée sans empreinte (écrite à la main, pour changer de
            # clé) reçoit celle de cette installation du même numéro.
            if inscrite > version or (inscrite == version and "empreinte" in ancien):
                lien = ancien
        nouveau = dict(self._registre)
        nouveau[manifest.app_id] = lien
        try:
            ecrire_registre_apps(self._chemin_registre, nouveau)
        except OSError as exc:
            print(f"  [ERREUR] Registre des apps non enregistré ({exc!r:.120}) : "
                  f"installation refusée.")
            return False
        self._registre = nouveau
        return True

    def _sdk_compatible(self, manifest: AppManifest) -> bool:
        """Compatibilité du SDK, dont la version est celle d'AXIS-OS (voir
        axis_os/__init__.py ; versions au format SemVer). Vérifiée avant le
        reste : une app écrite pour une version incompatible peut attendre
        un autre format pour HardwareRequirements ou Capabilities."""
        if manifest.sdk_version_min is not None:
            from axis_os import __version__ as axis_os_version, SemVer
            required = SemVer.parse(manifest.sdk_version_min)
            current  = SemVer.parse(axis_os_version)
            if not required.is_compatible_with(current):
                print(
                    f"  [ERREUR] Incompatible avec AXIS-OS {current} — "
                    f"cette app requiert {required} (SDK mineur ou majeur "
                    f"différent). Mettez à jour l'app ou installez-la sur "
                    f"une version d'AXIS-OS compatible."
                )
                return False
        if manifest.min_sdk_version is not None:
            from axis_os import SemVer
            from axis_os.store.sdk.version import API_VERSION
            required = SemVer.parse(manifest.min_sdk_version)
            current = SemVer.parse(API_VERSION)
            if required.major != current.major or current < required:
                print(
                    f"  [ERREUR] API du SDK {current} trop ancienne ou incompatible — "
                    f"cette app requiert l'API {required} (même version majeure, au "
                    f"moins cette version). Mettez à jour AXIS-OS.")
                return False
        return True

    def _cle_approuvee(self, cle: Optional[str]) -> bool:
        """Vrai si `cle` (clé publique du développeur, déjà vérifiée contre
        la signature) est acceptée : clé plateforme, ou clé du fichier des
        clés approuvées. Sans ce fichier (hors production), toute clé
        est acceptée."""
        if self._cles_approuvees is None:
            return True
        if cle is not None and (cle in self._cles_approuvees
                                or cle == self._platform_public_key):
            return True
        try:
            qui = public_key_fingerprint(cle)
        except Exception:                 # noqa: BLE001 — clé malformée : rien à montrer
            qui = "clé illisible"
        print(f"  [ERREUR] Clé de développeur non approuvée sur ce robot ({qui}) : "
              f"absente du fichier des clés approuvées.")
        return False

    def _lire_paquet(self, source: Any, manifeste: Any, code_signature: str,
                     developer_public_key: str, origine: str):
        """Étapes communes à install_source() et update_source(), sans rien
        exécuter de l'app : signature du paquet (manifeste déclaré + code),
        clé approuvée, manifeste relu et validé, version stricte, SDK. Rend
        (manifeste, spec, version, empreinte_paquet), ou None une fois la
        raison affichée."""
        from axis_os.store.sdk.signing import verify_app_package, SignatureError
        from axis_os.store.runtime.manifeste_json import manifeste_depuis_texte, MessageRefuse

        etiquette = "".join(c if c.isalnum() else "_" for c in origine)[:64] or "app"
        if not isinstance(source, str) or not isinstance(manifeste, str):
            print("  [ERREUR] Source et manifeste déclaré de l'app attendus sous forme de "
                  "texte.")
            return None
        try:
            valide = verify_app_package(manifeste, source, code_signature, developer_public_key)
        except SignatureError as exc:
            print(f"  [ERREUR] Signature ou clé malformée : {exc}")
            return None
        if not valide:
            print("  [ERREUR] Signature invalide — le manifeste déclaré ou le code ne "
                  "correspond pas à sa signature (modifié après signature, mauvaise clé, ou "
                  "code signé seul : signer le paquet avec sign_app_package).")
            return None
        if not self._cle_approuvee(developer_public_key):
            return None
        try:
            manifest = manifeste_depuis_texte(manifeste)
        except (MessageRefuse, KeyError, ValueError) as exc:
            print(f"  [ERREUR] Manifeste déclaré refusé : {_affichable(exc, 300)}")
            return None
        manifest.code_signature = code_signature
        manifest.developer_public_key = developer_public_key

        print(f"  App       : '{_affichable(manifest.name)}' v{_affichable(manifest.version)} "
              f"({_affichable(manifest.app_id)})")
        print(f"  Auteur déclaré : {_affichable(manifest.author)}")
        errors = manifest.validate()
        if errors:
            for e in errors:
                print(f"  [ERREUR] {_affichable(e, 300)}")
            return None
        try:
            version = SemVer.parse(manifest.version)
        except ValueError as exc:
            print(f"  [ERREUR] {_affichable(exc, 300)}")
            return None
        if not self._sdk_compatible(manifest):
            return None
        spec = {"genre": "source", "module": f"axis_app_{etiquette}", "classe": None,
                "source": source, "fichier": f"<{etiquette}>", "paquet": None}
        empreinte = empreinte_paquet(manifest, source)
        if empreinte is None:
            return None
        return manifest, spec, version, empreinte

    def install_source(self, source: str, *, manifeste: str, code_signature: str,
                       developer_public_key: str, origine: str = "app") -> bool:
        """Installe une app à partir de son code source, de son manifeste
        déclaré et de leur signature commune (signing.sign_app_package), sans
        jamais exécuter son code avant que l'humain ait accepté.

        C'est le seul chemin d'installation en profil PRODUCTION. Ordre :
          1. la signature est vérifiée sur le manifeste et le code (de
             simples octets), et la clé doit être approuvée ;
          2. le manifeste déclaré (texte JSON, manifeste_json.manifeste_texte)
             est relu champ par champ ; aucune ligne de l'app ne tourne pour
             cela, ni dans l'OS ni dans un processus d'app ;
          3. validate(), version SemVer stricte, SDK, puis le registre des
             apps : un app_id lié à une autre clé, ou une version antérieure
             à la plus récente déjà installée, est refusé ;
          4. les contrôles d'install() (plafonds des quotas, permissions
             SIGNATURE, capacités, matériel), la confirmation des permissions
             DANGEROUS, puis l'inscription au registre.

        L'OS applique ce manifeste-là ; ce que renvoie la méthode manifest()
        du code n'est jamais consulté. code_signature et developer_public_key
        sont ceux vérifiés ici. Une app déjà installée sous le même app_id se
        met à jour par update_source().
        """
        if not self._verrou_installation.acquire(blocking=False):
            print("  [ERREUR] Une autre installation est en cours sur ce robot : refusé.")
            return False
        try:
            print(f"\n[PlayStore] Installation depuis la source ({_affichable(origine)})")
            lu = self._lire_paquet(source, manifeste, code_signature, developer_public_key,
                                   origine)
            if lu is None:
                return False
            manifest, spec, version, empreinte = lu
            if manifest.app_id in self._apps:
                print(f"  [ERREUR] '{_affichable(manifest.app_id)}' est déjà installée : "
                      f"la mettre à jour par update_source().")
                return False
            if not self._verifier_registre(manifest, version, empreinte):
                return False
            return self._terminer_installation(
                manifest, None, spec, None,
                avant_enregistrement=lambda: self._lier(manifest, version, empreinte))
        finally:
            self._verrou_installation.release()

    def update_source(self, source: str, *, manifeste: str, code_signature: str,
                      developer_public_key: str, origine: str = "app") -> bool:
        """Met à jour une app installée, depuis son paquet signé : le chemin
        des mises à jour en PRODUCTION, où update_app() n'est pas permis.

        Mêmes vérifications qu'install_source(), sans exécution avant
        l'humain, et en plus : la clé doit être celle de la version installée
        et celle à laquelle l'app_id est lié, et la version STRICTEMENT plus
        récente que la version installée (pas de retour en arrière, pas
        d'allow_downgrade). Ensuite, comme update_app() : refus si une
        mission est en cours, ancienne version arrêtée, état de facturation
        conservé, toutes les vérifications et la confirmation d'une
        installation neuve. Si elles échouent, l'ancienne version reste
        installée, arrêtée. L'app n'est pas redémarrée d'office. Les
        missions sont refusées pendant toute la mise à jour, confirmation
        humaine comprise.
        """
        if not self._verrou_installation.acquire(blocking=False):
            print("  [ERREUR] Une autre installation est en cours sur ce robot : refusé.")
            return False
        try:
            print(f"\n[PlayStore] Mise à jour depuis la source ({_affichable(origine)})")
            lu = self._lire_paquet(source, manifeste, code_signature, developer_public_key,
                                   origine)
            if lu is None:
                return False
            manifest, spec, version, empreinte = lu
            app_id = manifest.app_id
            entree = self._apps.get(app_id)
            if entree is None:
                print(f"  [ERREUR] '{_affichable(app_id)}' n'est pas installée : "
                      f"install_source() pour une première installation.")
                return False
            ancienne = entree.manifest
            if ancienne.code_signature is None or ancienne.developer_public_key != developer_public_key:
                avant = ("aucune (app non signée)" if ancienne.code_signature is None
                         else public_key_fingerprint(ancienne.developer_public_key))
                print(f"  [ERREUR] Mise à jour signée par "
                      f"{public_key_fingerprint(developer_public_key)}, version installée "
                      f"signée par {avant} : seule la même clé peut mettre à jour une app.")
                return False
            try:
                installee = SemVer.parse(ancienne.version)
            except ValueError:
                print(f"  [ERREUR] Version installée illisible ({_affichable(ancienne.version)}) : "
                      f"mise à jour refusée.")
                return False
            if not version > installee:
                print(f"  [ERREUR] v{version} n'est pas strictement plus récente que la version "
                      f"installée (v{installee}) : mise à jour refusée.")
                return False
            if not self._verifier_registre(manifest, version, empreinte):
                return False
            # Tenu pendant toute la mise à jour : aucune mission ne démarre
            # pendant qu'on remplace le code de l'app.
            if not self._mission_lock.acquire(blocking=False):
                print(f"  [ERREUR] Une mission est en cours sur ce robot : mise à jour de "
                      f"'{_affichable(app_id)}' refusée.")
                return False
            self._maj_en_cours = app_id
            try:
                if entree.is_started:
                    print(f"  [PlayStore] Arrêt de v{installee} avant mise à jour...")
                    self.stop_app(app_id)
                facture_une_fois = entree.one_time_billed
                derniere_facture = entree.last_subscription_bill_time_s
                missions, revenu = entree.mission_count, entree.total_revenue
                print(f"  [PlayStore] Mise à jour de '{_affichable(app_id)}' : "
                      f"v{installee} → v{version}")
                if entree.app is not None:
                    print("  [ATTENTION] Version installée depuis un objet app : son état "
                          "d'instance n'est pas repris par la version installée depuis la source.")
                if not self._terminer_installation(
                        manifest, None, spec, None,
                        avant_enregistrement=lambda: self._lier(manifest, version, empreinte)):
                    print(f"  [ERREUR] La nouvelle version a échoué aux vérifications : "
                          f"l'ANCIENNE (v{installee}) reste installée, arrêtée.")
                    return False
                nouvelle = self._apps[app_id]
                nouvelle.one_time_billed = facture_une_fois
                nouvelle.last_subscription_bill_time_s = derniere_facture
                nouvelle.mission_count, nouvelle.total_revenue = missions, revenu
                return True
            finally:
                self._maj_en_cours = None
                self._mission_lock.release()
        finally:
            self._verrou_installation.release()

    # ── Cycle de vie ──────────────────────────────────────────────

    def start_app(self, app_id: str) -> bool:
        """Démarre une app installée en exécutant son hook on_start() dans
        la sandbox. Renvoie True si l'app s'est déclarée prête."""
        entry = self._get(app_id)
        if entry is None: return False
        if app_id == self._maj_en_cours:
            print(f"  [ERREUR] '{_affichable(app_id)}' est en cours de mise à jour : "
                  f"démarrage refusé.")
            return False

        # Exécuté dans la sandbox. Pourquoi : un audit de sécurité externe
        # (2026-09-14, voir docs/audit_externe_2026-09-14.md et
        # CHANGELOG.md) a montré que start_app() et stop_app() appelaient
        # on_start() et on_stop() directement dans le processus de l'OS,
        # avec un vrai RobotContext : accès complet au HAL, sans quota ni
        # délai. Le risque était réel : package_sorting appelle
        # ctx.move_arms(...) dans son on_start(). Même pont et même
        # isolation que run_mission() (voir _executer_hook).
        #
        # Ce RobotContext reste dans le processus de l'OS : il sert au
        # journal et au HALBridgeServer, qui exécute pour l'app les
        # commandes reçues par le pont. L'app ne le reçoit jamais.
        ctx = self._make_ctx(entry.manifest)
        ctx.log(f"Démarrage '{entry.manifest.name}'")
        # on_start (re)déclare le vocabulaire de l'app : celui d'un démarrage
        # précédent, réussi ou non, est d'abord retiré.
        registre_vocal = getattr(self._speech_service, "registry", None)
        if registre_vocal is not None:
            registre_vocal.retirer(app_id)
        try:
            # Un arrêt d'urgence interrompt aussi le démarrage : une app ne
            # démarre pas sous e-stop (voir `arret` dans SandboxRunner.run_app).
            sandbox_result, _ = self._executer_hook(entry, "on_start", ctx,
                                                    arret=self._estop_actif)
            if sandbox_result.success:
                ready = sandbox_result.result["pret"]
            else:
                kind = sandbox_result.error_kind
                reason = kind.name if kind else "INCONNU"
                print(f"  [SANDBOX] on_start arrêté par l'isolation : "
                      f"{sandbox_result.state.name} ({reason}) — {sandbox_result.error}")
                ready = False
        except Exception as exc:
            print(f"  [ERREUR] on_start: {exc}")
            ready = False

        # Comme pour une mission : un e-stop survenu pendant on_start, même
        # si l'app a eu le temps de répondre, la laisse non démarrée.
        ready = ready and not self._estop_actif()
        entry.is_started = ready
        ctx.log("Prête." if ready else "Échec initialisation.")
        return ready

    def run_mission(
        self, app_id: str, params: Optional[Dict[str, Any]] = None
    ) -> Optional[MissionResult]:
        """Exécute une mission de l'app dans la sandbox et la facture si
        elle a réellement abouti. Renvoie None si l'app n'est pas installée
        ou pas démarrée, et un MissionResult d'échec si la mission est
        refusée (e-stop actif, autre mission déjà en cours)."""
        entry = self._get(app_id)
        if entry is None: return None
        if not entry.is_started:
            print(f"  [ERREUR] App '{app_id}' non démarrée.")
            return None

        # Barrière critique, ajoutée après un test de charge et
        # d'intégration : aucune mission ne doit pouvoir s'exécuter, même
        # « avec succès », une fois qu'un e-stop global a été déclenché.
        # Sans ce contrôle, une app qui ne consulte jamais elle-même le
        # Watchdog pouvait continuer indéfiniment après une condition de
        # sécurité FATAL (le niveau d'alerte le plus grave du Watchdog).
        #
        # On consulte aussi self._robot.estop_active. Un audit de sécurité
        # externe a montré qu'un e-stop déclenché par le Supervisor (le
        # thread qui arrête le robot quand le Watchdog lui-même cesse de
        # fonctionner, donc précisément quand le Watchdog est mort ou
        # bloqué) ne positionnait pas watchdog.estop_fired, et restait
        # donc invisible ici.
        if self._estop_actif():
            mission_id = str(uuid.uuid4())[:8].upper()
            print(f"  [SECURITY] Mission {mission_id} refusée : "
                  f"e-stop actif sur le robot — {app_id}")
            return MissionResult(
                success=False, mission_id=mission_id, app_id=app_id,
                duration_s=0.0,
                error_message="Robot en état E-STOP — aucune mission autorisée.",
            )

        # Verrou global de mission (raison expliquée dans __init__).
        # acquire(blocking=False) : un appel concurrent est refusé tout de
        # suite avec un résultat explicite, jamais mis en attente en
        # silence derrière une mission déjà en cours.
        if not self._mission_lock.acquire(blocking=False):
            mission_id = str(uuid.uuid4())[:8].upper()
            print(f"  [SECURITY] Mission {mission_id} refusée : une autre mission, ou la "
                  f"mise à jour d'une app, est déjà en cours sur ce robot — {app_id}")
            return MissionResult(
                success=False, mission_id=mission_id, app_id=app_id,
                duration_s=0.0,
                error_message=(
                    "Une autre mission, ou la mise à jour d'une app "
                    "(update_source, confirmation comprise), est déjà en cours "
                    "sur ce robot — un seul robot ne peut exécuter qu'une "
                    "mission à la fois, quelle que soit l'app."
                ),
            )

        try:
            return self._run_mission_locked(entry, app_id, params)
        finally:
            self._mission_lock.release()

    def _run_mission_locked(
        self, entry: "InstalledApp", app_id: str, params: Optional[Dict[str, Any]]
    ) -> Optional[MissionResult]:
        """Corps de run_mission(), exécuté sous _mission_lock (contexte dans
        run_mission()). Il est extrait dans sa propre méthode pour que le
        verrou couvre tout le corps par un simple try/finally, sans
        dupliquer sa logique."""

        mission_id = str(uuid.uuid4())[:8].upper()
        params     = params or {}
        ctx        = self._make_ctx(entry.manifest)
        print(f"\n[PlayStore] ▶ Mission {mission_id} | app={app_id}")

        t_start = time.monotonic()

        # Isolation par processus séparé. Pourquoi : à l'origine,
        # entry.app.run_mission(ctx, params) était appelé directement dans
        # ce processus, sans aucune des protections (quotas de CPU, de
        # mémoire et de durée) que le reste du projet suppose actives pour
        # toute app. Le pont HALBridgeServer/HALBridgeClient permet à
        # l'app isolée d'appeler move_arms/move_legs/log/get_imu exactement
        # comme avec un RobotContext, sans qu'aucune app ait à changer.
        # Un arrêt d'urgence survenu pendant la mission l'interrompt : voir
        # `arret` dans SandboxRunner.run_app.
        sandbox_result, bridge_server = self._executer_hook(
            entry, "run_mission", ctx, params, arret=self._estop_actif)

        if sandbox_result.success and sandbox_result.result.get("mission") is not None:
            # Reconstruit champ par champ à partir du JSON vérifié
            # (processus_app.SCHEMA_MISSION) : aucun objet de l'app.
            m = sandbox_result.result["mission"]
            result = MissionResult(
                success=m["success"], mission_id=mission_id, app_id=app_id,
                duration_s=time.monotonic() - t_start,
                items_processed=m["items_processed"] or 0,
                error_message=m["error_message"],
                metrics=m["metrics"] if isinstance(m["metrics"], dict) else {},
                log=list(m["log"] or []),
            )
        elif sandbox_result.success:
            # L'app a renvoyé None au lieu d'un MissionResult. Une
            # PermissionError ou une EstopActiveError n'arrive pas ici :
            # le processus de l'app les convertit en MissionResult d'échec
            # (hote_app._executer), pour garder la distinction entre refus
            # de sécurité et bug de l'app.
            result = MissionResult(
                success=False, mission_id=mission_id, app_id=app_id,
                duration_s=time.monotonic() - t_start,
                error_message="Mission sandboxée n'a retourné aucun résultat.",
            )
        else:
            # La sandbox a signalé un échec : délai ou quota dépassé,
            # plantage, violation du protocole du pont, app impossible à
            # recharger, ou exception levée par l'app (APPLICATION_ERROR).
            # Le SandboxErrorKind indique précisément la cause.
            kind = sandbox_result.error_kind
            reason = kind.name if kind else "INCONNU"
            print(f"  [SANDBOX] Mission arrêtée par l'isolation : "
                  f"{sandbox_result.state.name} ({reason})")
            result = MissionResult(
                success=False, mission_id=mission_id, app_id=app_id,
                duration_s=time.monotonic() - t_start,
                error_message=(f"Isolation sandbox : {sandbox_result.state.name} "
                               f"— {sandbox_result.error}"),
            )

        if result:
            result.mission_id = mission_id
            result.app_id     = app_id
            result.duration_s = time.monotonic() - t_start

        # Arrêt d'urgence pendant la mission : elle est notée interrompue,
        # quoi qu'en dise l'app, et donc jamais facturée. Lu APRÈS le hook :
        # run_mission() refuse déjà toute mission sous e-stop, donc un e-stop
        # vu ici est survenu pendant celle-ci (y compris déclenché par l'app
        # elle-même, ou juste au moment où elle rendait success=True).
        if result and (self._estop_actif()
                       or sandbox_result.error_kind is SandboxErrorKind.ARRET_URGENCE):
            result.success = False
            result.interrompue_par_arret_urgence = True
            result.error_message = MOTIF_ARRET_URGENCE + (
                f" — {result.error_message}" if result.error_message else "")

        # Facturation. Pourquoi cette double condition : un audit de
        # sécurité externe a montré que la facturation reposait
        # uniquement sur result.success, une valeur que l'app déclare
        # elle-même depuis le processus enfant. Il faut désormais en plus
        # qu'au moins une commande motrice ait réellement abouti
        # (bridge_server.had_any_motor_success, tenu dans le processus
        # parent, hors de portée de l'app). Cela élimine le cas le plus
        # grave : une app qui ne bouge jamais le robot mais renvoie
        # toujours success=True pour déclencher la facturation du client.
        # Limite connue : le contrôle ne vérifie pas que le mouvement
        # correspond à la mission déclarée, donc il ne couvre pas tous les
        # mensonges possibles ; il supprime la fraude la plus simple et la
        # plus rentable à automatiser.
        if result and result.success and bridge_server.had_any_motor_success:
            entry.mission_count += 1
            billing = self._bill(entry, mission_id)
            if billing:
                entry.total_revenue += billing.developer_eur
                self._billing.append(billing)
                print(f"  {billing}")

        print(f"  {result}")
        return result

    def stop_app(self, app_id: str) -> None:
        """Arrête une app démarrée en exécutant son hook on_stop() dans la
        sandbox. Sans effet si l'app n'est pas installée ou pas démarrée.
        L'app est marquée arrêtée même si on_stop() échoue."""
        entry = self._get(app_id)
        if entry is None or not entry.is_started: return

        # Exécuté dans la sandbox, pour la même raison que start_app().
        ctx = self._make_ctx(entry.manifest)
        try:
            # Sous e-stop, on_stop aussi est interrompu : il ne garde ni les
            # capteurs ni les verrous jusqu'à son quota.
            sandbox_result, _ = self._executer_hook(entry, "on_stop", ctx,
                                                    arret=self._estop_actif)
            if not sandbox_result.success:
                kind = sandbox_result.error_kind
                reason = kind.name if kind else "INCONNU"
                print(f"  [SANDBOX] on_stop arrêté par l'isolation : "
                      f"{sandbox_result.state.name} ({reason}) — {sandbox_result.error}")
        except Exception as exc:
            print(f"  [ERREUR] on_stop: {exc}")

        entry.is_started = False
        # Le vocabulaire de l'app part avec elle ; son prochain on_start (ou
        # sa nouvelle version) le redéclare.
        registre = getattr(self._speech_service, "registry", None)
        if registre is not None:
            registre.retirer(app_id)

    def update_app(
        self, new_app: RobotApp, *, allow_downgrade: bool = False,
    ) -> bool:
        """
        Met à jour une app déjà installée : même app_id, nouvelle instance
        ou nouvelle version.

        Pourquoi cette méthode : avant elle, rien n'empêchait install()
        d'écraser en silence une app déjà installée sous le même app_id.
        Une version en cours d'exécution n'était pas arrêtée proprement,
        le sens de la mise à jour n'était pas vérifié (une version plus
        ancienne pouvait remplacer une plus récente sans avertissement),
        et l'état de facturation (one_time_billed,
        last_subscription_bill_time_s) repartait de zéro comme pour une
        installation neuve : une app ONE_TIME déjà payée aurait été
        refacturée à sa prochaine mission après une simple mise à jour.

        Le comportement s'inspire du cycle réel d'une mise à jour sous
        Android (son gestionnaire de paquets, PackageManager) :
          1. Tout retour à une version antérieure (downgrade : nouvelle
             version < version installée) est refusé par défaut, comme
             le fait Android avec l'erreur INSTALL_FAILED_VERSION_DOWNGRADE.
             C'est une protection de sécurité connue : sans elle, un
             attaquant pourrait faire « mettre à jour » une app vers une
             ancienne version contenant une faille corrigée depuis.
             `allow_downgrade=True` lève ce refus explicitement (comme
             l'option `-d` de `adb install`, l'outil Android
             d'installation en ligne de commande) ; ce n'est jamais le
             comportement par défaut, et c'est sans effet quand le
             runtime tient un registre des apps (registre_apps).
          2. La mise à jour est refusée si une mission est en cours sur
             ce robot (self._mission_lock déjà pris), quelle que soit
             l'app concernée. C'est le principe déjà appliqué à
             run_mission() : un seul robot, une seule activité à la fois.
             Remplacer le code d'une app pendant qu'une mission, même
             d'une autre app, contrôle physiquement le robot serait
             dangereux.
          3. L'ancienne version est arrêtée proprement si elle est
             démarrée, par stop_app() tel quel : mêmes garanties
             d'isolation que l'arrêt normal, jamais un raccourci qui
             sauterait on_stop().
          4. La nouvelle version est installée par install(), qui
             applique toutes les vérifications existantes (signature,
             permissions, dont la confirmation des permissions DANGEROUS,
             capacités, compatibilité du SDK, ProtectionLevel.SIGNATURE)
             exactement comme pour une première installation. Une mise à
             jour n'est jamais un chemin plus permissif qu'une
             installation neuve.
          5. L'état de facturation de l'ancienne entrée est recopié sur
             la nouvelle (one_time_billed, last_subscription_bill_time_s) :
             une mise à jour n'est pas une réinstallation au sens
             commercial, l'app reste « la même » pour un utilisateur qui
             a déjà payé.

        L'app n'est pas redémarrée automatiquement après la mise à jour,
        même si l'ancienne version l'était. C'est délibéré et conforme au
        comportement réel d'Android (ACTION_MY_PACKAGE_REPLACED prévient
        l'app remplacée, mais c'est à elle, ou ici à l'appelant, de
        décider explicitement de redémarrer) : appeler start_app(app_id)
        après update_app() reste la responsabilité de l'appelant.

        Si la nouvelle version échoue aux vérifications d'install(),
        l'ancienne reste installée, mais arrêtée si elle tournait.
        Essais : test/test_app_update.py.
        """
        if self._profil is Profil.PRODUCTION:
            # Refusé avant tout appel à l'objet (manifest() tournerait dans
            # l'OS) et avant d'arrêter l'ancienne version, qui reste en
            # service.
            print("  [ERREUR] update_app() : refusé en profil PRODUCTION, où une app "
                  "s'installe et se met à jour depuis son paquet signé : utiliser "
                  "update_source().")
            return False
        new_manifest = new_app.manifest()
        app_id = new_manifest.app_id

        existing_entry = self._get(app_id)
        if existing_entry is None:
            print(f"  [ERREUR] update_app() : aucune app installée avec "
                  f"l'app_id '{app_id}' — utilisez install() pour une "
                  f"première installation.")
            return False

        try:
            new_version = SemVer.parse(new_manifest.version)
            old_version = SemVer.parse(existing_entry.manifest.version)
        except ValueError as e:
            print(f"  [ERREUR] update_app() : version invalide — {e}")
            return False

        if new_version < old_version and self._registre is not None:
            # Avec un registre des apps, aucun retour en arrière (même
            # allow_downgrade) : refusé ici, avant d'arrêter l'ancienne version.
            print(f"  [ERREUR] update_app() : v{new_version} est antérieure à la version "
                  f"installée (v{old_version}) et ce robot tient un registre des apps : "
                  f"retour en arrière refusé, allow_downgrade est sans effet.")
            return False
        if new_version < old_version and not allow_downgrade:
            print(
                f"  [ERREUR] update_app() : la nouvelle version "
                f"({new_version}) est ANTÉRIEURE à la version installée "
                f"({old_version}) — downgrade refusé par défaut (risque "
                f"de sécurité connu : revenir à une version contenant une "
                f"faille déjà corrigée). Passez allow_downgrade=True pour "
                f"forcer ce remplacement explicitement."
            )
            return False

        if not self._mission_lock.acquire(blocking=False):
            print(
                f"  [ERREUR] update_app() : une mission est actuellement "
                f"en cours sur ce robot — impossible de mettre à jour "
                f"'{app_id}' tant qu'elle n'est pas terminée."
            )
            return False
        self._mission_lock.release()

        self._maj_en_cours = app_id
        try:
            return self._remplacer_objet(existing_entry, new_app, app_id, old_version,
                                         new_version)
        finally:
            self._maj_en_cours = None

    def _remplacer_objet(self, existing_entry, new_app, app_id, old_version,
                         new_version) -> bool:
        """Suite d'update_app(), pendant laquelle start_app(app_id) est refusé."""
        was_started = existing_entry.is_started
        if was_started:
            print(f"  [PlayStore] Arrêt de l'ancienne version de "
                  f"'{app_id}' (v{old_version}) avant mise à jour...")
            self.stop_app(app_id)

        # On garde l'état de facturation avant qu'install() ne crée une
        # nouvelle InstalledApp aux valeurs par défaut (raison : étape 5
        # de la docstring de update_app()).
        preserved_one_time_billed = existing_entry.one_time_billed
        preserved_last_subscription_bill_time_s = existing_entry.last_subscription_bill_time_s
        missions, revenu = existing_entry.mission_count, existing_entry.total_revenue

        print(f"  [PlayStore] Mise à jour de '{app_id}' : "
              f"v{old_version} → v{new_version}")
        installed = self.install(new_app)
        if not installed:
            print(f"  [ERREUR] update_app() : la nouvelle version de "
                  f"'{app_id}' a échoué aux vérifications d'install() — "
                  f"l'ANCIENNE version reste installée mais ARRÊTÉE "
                  f"(voir ci-dessus). Réinstallez-la explicitement ou "
                  f"corrigez la nouvelle version avant de réessayer.")
            return False

        new_entry = self._get(app_id)
        if new_entry is not None:
            new_entry.one_time_billed = preserved_one_time_billed
            new_entry.last_subscription_bill_time_s = preserved_last_subscription_bill_time_s
            new_entry.mission_count, new_entry.total_revenue = missions, revenu

        return True

    # ── Rapport financier ─────────────────────────────────────────

    def billing_summary(self) -> Dict[str, Any]:
        """Résumé financier de ce runtime : totaux (montant brut,
        commission, part développeur) sur toutes les facturations émises,
        puis détail par app installée. « total_missions » compte les
        facturations émises, pas toutes les missions exécutées."""
        total_gross = sum(b.gross_amount_eur for b in self._billing)
        total_comm  = sum(b.commission_eur   for b in self._billing)
        total_dev   = sum(b.developer_eur    for b in self._billing)
        return {
            "runtime_id":      self._id,
            "total_missions":  len(self._billing),
            "total_gross_eur": round(total_gross, 2),
            "commission_eur":  round(total_comm, 2),
            "developer_eur":   round(total_dev, 2),
            "commission_rate": "30%",
            "apps_installed":  len(self._apps),
            "per_app": {
                aid: {"missions": e.mission_count,
                      "revenue":  round(e.total_revenue, 2),
                      "commission": round(e.commission_due, 2)}
                for aid, e in self._apps.items()
            }
        }

    # ── Helpers ───────────────────────────────────────────────────

    def _available_capabilities(self) -> set:
        """
        Déduit les capacités fonctionnelles (Capability) disponibles sur ce
        robot à partir des capacités matérielles (RobotCapability) déjà
        exposées par le HAL (get_capabilities()). Ce n'est pas une
        nouvelle source de vérité indépendante : deux déclarations du même
        robot ne peuvent ainsi jamais diverger.

        Correspondance volontairement prudente pour la perception :
        RobotCapability.CAMERA (générique, ne distingue pas une caméra
        couleur RGB d'une caméra de profondeur) ne donne que
        Capability.CAMERA_RGB, jamais CAMERA_DEPTH. Annoncer la profondeur
        sans information plus précise serait mentir à l'installation.

        WALK et GRASP_SINGLE sont déduites de LEGS et ARMS. Correction
        faite après un essai réel sur les trois apps de référence, qui
        bougent déjà bras et jambes directement par move_arms() et
        move_legs(), donc par un mouvement physique réel via le HAL, pas
        hypothétique. Une version précédente de cette correspondance les
        excluait au motif qu'« aucun service système ne les fournit
        encore » : elle confondait la capacité de mouvement de base, déjà
        réelle et exercée par les apps existantes, avec un futur service
        intelligent de haut niveau (planification de la marche, préhension
        guidée par la vision), qui, lui, n'existe effectivement pas encore.
        GRASP_DUAL demande les deux bras coordonnés : elle n'est pas
        déduite d'un simple ARMS générique, et aucune app de référence
        actuelle ne l'utilise ni n'en a besoin.
        """
        robot_caps = self._robot.get_capabilities()
        available = set()
        if robot_caps.has(RobotCapability.CAMERA):
            available.add(Capability.CAMERA_RGB)
        if robot_caps.has(RobotCapability.MICROPHONE):
            available.add(Capability.SPEECH_IN)
        if robot_caps.has(RobotCapability.SPEAKER):
            available.add(Capability.SPEECH_OUT)
        if robot_caps.has(RobotCapability.LEGS):
            available.add(Capability.WALK)
        if robot_caps.has(RobotCapability.ARMS):
            available.add(Capability.GRASP_SINGLE)

        # Capacités logicielles : elles ne viennent pas du matériel mais
        # des services effectivement présents sur ce robot. Une mémoire
        # persistante n'est pas un capteur : elle existe si le noyau
        # fournit le service, et pas autrement.
        #
        # Oubli constaté : la table ne traduisait que des capacités
        # matérielles, donc MEMORY_PERSISTENT n'était jamais disponible et
        # toute app qui l'exigeait était refusée à l'installation, sans
        # que rien n'indique pourquoi la mission suivante échouait.
        if self._memory_service is not None:
            available.add(Capability.MEMORY_PERSISTENT)
        return available

    def _estop_actif(self) -> bool:
        """Vrai si un arrêt d'urgence est actif. Deux sources : le Watchdog
        (estop_fired) et le HAL (estop_active), que positionne aussi le
        Supervisor quand le Watchdog est mort ou bloqué. Aucun réarmement
        n'existe : seul un redémarrage de l'OS le remet à zéro."""
        return bool(self._watchdog.estop_fired or self._robot.estop_active)

    def _executer_hook(self, entry: "InstalledApp", hook: str, ctx: RobotContext,
                       params: Optional[Dict[str, Any]] = None,
                       arret: Optional[Callable[[], bool]] = None):
        """Exécute un hook de l'app dans son processus neuf et rend
        (SandboxResult, HALBridgeServer). Le serveur garde, côté OS, le
        compte des commandes motrices réussies (facturation)."""
        bridge_server = HALBridgeServer(ctx)
        if entry.spec is None:
            return SandboxResult(
                False, SandboxState.CRASHED,
                error=f"App impossible à recharger dans son processus : {entry.spec_erreur}",
                error_kind=SandboxErrorKind.SANDBOX_ERROR), bridge_server
        # En production, quotas CPU et mémoire obligatoires (psutil) : voir
        # `strict` dans SandboxRunner.
        runner = SandboxRunner(entry.manifest.app_id, entry.manifest.quota,
                               clock=self._clock,
                               strict=self._profil is Profil.PRODUCTION)
        # L'état de l'instance est relu à chaque appel, comme le faisait
        # pickle : un attribut changé par l'opérateur entre deux missions
        # est vu par la suivante.
        etat = etat_json(entry.app) if entry.app is not None else None
        # Échéance du hook, lue par ctx.safety_status() (temps restant).
        ctx._set_deadline(self._clock() + entry.manifest.quota.max_runtime_s)
        app_id = entry.manifest.app_id
        self._ctx_courants.setdefault(app_id, []).append(ctx)
        try:
            result = runner.run_app(entry.spec, hook, serveur=bridge_server,
                                    params=params, etat=etat,
                                    tolerance_lenteur=HALBridgeClient.TOLERANCE_LENTEUR,
                                    arret=arret)
        finally:
            # Une action longue encore servie pour ce hook (go_to abandonné
            # après un arrêt d'urgence…) s'arrête au pas suivant.
            ctx._fermer()
            courants = self._ctx_courants.get(app_id, [])
            if ctx in courants:
                courants.remove(ctx)
            if not courants:
                self._ctx_courants.pop(app_id, None)
            if ctx._progression:
                self._progressions[app_id] = ctx._progression[-1]
        return result, bridge_server

    def request_stop(self, app_id: str, *, source: str = "human") -> bool:
        """Demande l'arrêt de la mission (ou du hook) en cours de l'app :
        source « human » (un opérateur) ou « os ». Coopératif : l'app le
        voit par ctx.should_stop() / check_cancelled(), et ses actions
        longues en cours rendent la main (CANCELLED). Rend False si aucun
        hook de l'app ne tourne. Ne remplace pas l'arrêt forcé : voir
        emergency_stop et le quota de durée."""
        if source not in ("human", "os"):
            raise ValueError(f"source d'arrêt inconnue : {source!r}")
        courants = list(self._ctx_courants.get(app_id, []))
        for ctx in courants:
            # Tous les hooks en cours de l'app : une demande d'arrêt ne doit
            # pas se perdre parce qu'un autre hook a commencé entre-temps.
            ctx._demander_arret(source)
        return bool(courants)

    def get_progress(self, app_id: str):
        """Dernière progression rapportée par l'app (Progress : fraction,
        message, timestamp), pendant le hook en cours ou au dernier hook ;
        None si elle n'en a jamais rapporté."""
        rapports = [c._progression[-1] for c in self._ctx_courants.get(app_id, [])
                    if c._progression]
        if rapports:
            return max(rapports, key=lambda p: p.timestamp)
        return self._progressions.get(app_id)

    def _make_ctx(self, manifest: AppManifest) -> RobotContext:
        """Construit le RobotContext d'une app (l'objet par lequel elle
        commande le robot), avec le jeton de session de ce runtime,
        l'horloge du robot et les services système."""
        ctx = RobotContext(
            self._robot, self._watchdog, manifest,
            _session_token=self._session_token,
            _runtime_token_holder=self._session_token,
        )
        # Les services sont transmis à toutes les apps : c'est la
        # permission déclarée dans le manifeste qui décide du droit de
        # s'en servir, pas la présence ou l'absence du service. Un service
        # absent (robot sans caméra, sans jambes) donne une réponse vide
        # et explicite, jamais une erreur obscure.
        ctx._set_clock(self._clock)
        ctx._set_os_info(self._profil.name)
        ctx._attach_services(perception=self._perception_service,
                             locomotion=self._locomotion_service,
                             speech=self._speech_service,
                             memory=self._memory_service,
                             navigation=self._navigation_service,
                             manipulation=self._manipulation_service)
        ctx._set_human_interface(self._human_interface)
        ctx._set_voice(self._speech_synthesizer)
        ctx._set_settings(self._reglages.get(manifest.app_id, {}))
        return ctx

    def set_setting(self, app_id: str, name: str, value) -> bool:
        """Réglage choisi par l'utilisateur pour une app installée, validé
        contre la déclaration de son manifeste (type, bornes, choix). Rend
        False, sans rien changer, si l'app ou le réglage est inconnu ou si
        la valeur ne respecte pas la déclaration. Gardé en mémoire : vu par
        le hook suivant de l'app."""
        entry = self._get(app_id)
        if entry is None:
            return False
        spec = next((s for s in entry.manifest.settings if s.name == name), None)
        if spec is None:
            print(f"  [ERREUR] réglage « {name} » non déclaré par {app_id}")
            return False
        try:
            spec.check(value)
        except ValueError as exc:
            print(f"  [ERREUR] réglage « {name} » de {app_id} refusé : {exc}")
            return False
        self._reglages.setdefault(app_id, {})[name] = value
        return True

    def _get(self, app_id: str) -> Optional[InstalledApp]:
        entry = self._apps.get(app_id)
        if entry is None:
            print(f"  [ERREUR] App '{app_id}' non installée.")
        return entry

    SUBSCRIPTION_PERIOD_S = 30 * 24 * 3600  # 30 jours en secondes ; voir _bill()

    def _read_billing_flag(self, app_id: str, key: str, default):
        """
        Lit un indicateur de facturation dans MemoryService. N'est appelée
        que si self._memory_service n'est pas None (voir install()).

        Renvoie `default` quand la lecture échoue (clé absente, service
        non démarré ou qui signale une erreur) plutôt que de lever une
        exception : un problème de lecture de l'état persisté ne doit
        jamais empêcher une app de s'installer. Au pire, l'app est traitée
        comme neuve pour cet app_id précis, ce qui n'est jamais pire que
        le comportement sans memory_service.
        """
        try:
            result = self._memory_service.app_load(f"billing::{app_id}", key)
        except Exception:                 # noqa: BLE001 — voir la docstring
            return default
        if not result.ok:
            return default
        valeur = result.value
        # Type attendu vérifié : une valeur d'un autre type (fichier abîmé,
        # écrit par autre chose que _write_billing_flag) ne doit ni faire
        # lever _bill() ni fausser la facturation.
        if key == "one_time_billed":
            return valeur if isinstance(valeur, bool) else default
        if key == "last_subscription_bill_time_s":
            ok = (isinstance(valeur, (int, float)) and not isinstance(valeur, bool)
                  and math.isfinite(valeur) and valeur >= 0)
            if not ok:
                print(f"  [ATTENTION] date de facturation illisible pour "
                      f"{_affichable(app_id)} : ignorée")
                return default
            # Une date dans le futur (horloge en retard au démarrage) est
            # gardée : elle empêche une nouvelle facturation, à l'avantage du
            # client, au lieu de faire refacturer l'abonnement.
            return float(valeur)
        return default

    def _write_billing_flag(self, app_id: str, key: str, value) -> None:
        """Écrit un indicateur de facturation dans MemoryService ; ne fait
        rien si self._memory_service est None. Un échec d'écriture n'est
        jamais propagé : la facturation en mémoire (InstalledApp, déjà mise
        à jour par l'appelant avant cet appel) reste correcte pour la
        session en cours même si la persistance échoue. Seule la survie au
        prochain redémarrage serait perdue, pas la session actuelle."""
        if self._memory_service is None:
            return
        try:
            self._memory_service.app_store(f"billing::{app_id}", key, value)
        except Exception as exc:          # noqa: BLE001 — voir la docstring
            print(f"  [ATTENTION] état de facturation non enregistré ({exc!r:.120})")

    def _bill(self, entry: InstalledApp, mission_id: str) -> Optional[BillingReport]:
        """
        Facture une mission selon le modèle de prix (pricing_model) de
        l'app. Renvoie None quand il n'y a rien à facturer.

        Pourquoi : un audit de sécurité externe a montré qu'avant ce
        correctif tous les modèles étaient facturés comme PAY_PER_MISSION
        (le prix complet à chaque mission), une erreur financière directe
        au détriment du client. Règles appliquées désormais :
          - FREE (gratuit) : jamais facturé (inchangé).
          - PAY_PER_MISSION (paiement à la mission) : facturé à chaque
            mission (comportement historique, correct pour ce modèle).
          - ONE_TIME (achat unique) : facturé une seule fois, à la
            première mission réussie après l'installation, et plus jamais
            ensuite, quel que soit le nombre de missions suivantes.
          - SUBSCRIPTION (abonnement) : facturé une fois par période de
            30 jours glissants depuis la dernière facturation, jamais à
            chaque mission. Une mission qui tombe dans la même période
            qu'une facturation déjà émise ne produit aucun BillingReport
            supplémentaire.

        one_time_billed et last_subscription_bill_time_s sont désormais
        enregistrés dans MemoryService quand il est fourni (voir
        _write_billing_flag). Cela comble le trou relevé par deux audits
        externes successifs : sans ce branchement, cet état ne survivait
        jamais à un redémarrage du processus (historique complet dans le
        commentaire des champs de facturation d'InstalledApp).
        """
        m = entry.manifest
        if m.pricing_model == PricingModel.FREE:
            return None

        if m.pricing_model == PricingModel.ONE_TIME:
            if entry.one_time_billed:
                return None
            entry.one_time_billed = True
            self._write_billing_flag(m.app_id, "one_time_billed", True)

        elif m.pricing_model == PricingModel.SUBSCRIPTION:
            # time.time() (horloge murale), pas time.monotonic() : voir le
            # commentaire des champs de facturation d'InstalledApp
            # (correction demandée par le second audit externe).
            now = time.time()
            last = entry.last_subscription_bill_time_s
            if last is not None and (now - last) < self.SUBSCRIPTION_PERIOD_S:
                return None
            entry.last_subscription_bill_time_s = now
            self._write_billing_flag(m.app_id, "last_subscription_bill_time_s", now)

        return BillingReport(
            mission_id=mission_id, app_id=m.app_id,
            pricing_model=m.pricing_model,
            gross_amount_eur=m.store_price_eur,
            commission_eur=m.platform_commission_eur,
            developer_eur=m.developer_revenue_eur,
        )
