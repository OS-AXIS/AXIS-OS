"""
AXIS-OS — Prise mesurée : géométrie et mesures de la prise à deux bras.

Ce que le service de manipulation (service.py) en tire, quand le robot lui
donne ses mesures (MesuresBras) :

  1. Portée : les mains peuvent-elles atteindre le milieu des deux faces
     opposées de l'objet (cinématique inverse, bornes de l'URDF) ? Sinon,
     refus OUT_OF_REACH, avec la distance à corriger. Jamais de serrage
     dans le vide.
  2. Distance à la table : les avant-bras restent au-dessus du plateau
     (`marge_plateau_m` plus le rayon de l'avant-bras), au-delà du bord de
     la table ; distance_ideale() la choisit pour l'approche.
  3. Mains au milieu des faces, en profondeur et en hauteur, d'après la
     position, la taille et le lacet de l'objet ; tolérances
     `tol_profondeur_m` et `tol_hauteur_m`, corrigées avant de serrer.
  4. Serrage mesuré : les bras se ferment jusqu'au CONTACT, détecté par
     l'effort (hausse du couple du roulis d'épaule depuis le début de la
     fermeture) ou par l'arrêt du rapprochement (la main ne suit plus sa
     consigne) ; puis un serrage supplémentaire, monté jusqu'à
     `effort_serrage_nm`, jamais au-delà de `effort_max_nm` ni d'une part
     `part_couple_garde` du couple maximal du garde-fou. Effort absent,
     non fini ou plus vieux que `age_max_effort_s` : on ne serre pas.

Repère : celui du bassin du robot (x devant, y à gauche, z en haut).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterable, Optional, Tuple

import numpy as np

from .cinematique import ModeleBras


@dataclass(frozen=True)
class ReglagesPrise:
    """Réglages de la prise mesurée (m, rad, N, N·m, s)."""
    # Demi-épaisseur d'une main : du repère de la main (rubber_hand de
    # l'URDF) à sa surface de contact. Mesurée sur le banc GPU du
    # 2026-10-28 : mains à 0,331 m l'une de l'autre sur un carton de
    # 0,24 m qu'elles touchaient.
    demi_main_m: float = 0.045
    # Point de contact : à `contact_main_m` du repère de la main, le long de
    # la main (axe x de rubber_hand), vers son bout. C'est là que la force à
    # la main s'estime (jacobienne) et là que la prise vise le milieu des
    # faces. Mesuré sur GPU (essai T1 du 5 novembre 2026, 16 h 06) : estimée
    # au repère de la main, la force valait 1,3 à 1,4 fois celle du juge
    # (8,4 N contre 6,4, sans décalage à zéro : une erreur de bras de levier,
    # pas de biais) ; elle la rejoint avec le contact à 11 cm, sur les deux
    # mains (couples du lacet d'épaule et du poignet, test_rejeu_prise_t1606).
    # Les mains visaient donc le centre du carton avec leur repère, et le
    # touchaient 11 cm plus loin, près de sa face arrière : le carton levé a
    # pivoté autour de la ligne des mains. À confirmer par les points de
    # contact PhysX relevés désormais par le banc (t1_prise.csv).
    contact_main_m: float = 0.11
    # Chaque main, à l'écart de sa face perçue avant de fermer. L'ouverture
    # dépasse l'erreur de position acceptée (`recalage_max_m`) d'au moins
    # `marge_ouverture_m` : sinon la main pourrait reposer déjà sur un carton
    # que la perception place à `recalage_max_m` de là, le zéro de la force
    # serait pris en appui et le contact vu trop tard (constaté en simulation
    # à 3 cm avec 4 cm d'ouverture : la main 2,5 cm dans la face). Vérifié à
    # la construction des réglages (__post_init__).
    jeu_ouverture_m: float = 0.05
    marge_ouverture_m: float = 0.02
    tol_profondeur_m: float = 0.025    # mains au milieu des faces, en profondeur
    # Point de prise : les mains au moins `recul_face_min_m` derrière la
    # face avant de l'objet (celle tournée vers le robot), idéalement au
    # milieu. Constaté sur GPU (essai T1, novembre 2026) : mains près du
    # bord avant d'un carton. Avec un carton de 20 cm, la tolérance
    # ci-dessus les garde déjà à 7.5 cm ; la règle borne la tolérance vers
    # l'avant d'un objet moins profond (voir avance_max).
    recul_face_min_m: float = 0.07
    # et en hauteur ; constaté sur GPU (2026-10-29) : mains 3,3 cm plus bas
    # que leurs cibles (bras qui s'affaissent), d'où une tolérance serrée et
    # la compensation ci-dessous.
    tol_hauteur_m: float = 0.015
    compensation_max_m: float = 0.10   # visée décalée d'au plus 10 cm par main pour compenser
    # 3 : prise en pendule, mains placées à la moitié de ce que la ligne des
    # mains admet en profondeur (tolerances_cg) ; bras affaissés de 0,25 rad,
    # pas de 20 à 100 ms, la 3e correction y amène (test_fermeture_geometrique).
    corrections_max: int = 3
    rayon_avant_bras_m: float = 0.035
    marge_plateau_m: float = 0.03      # sous l'avant-bras, au-dessus du plateau
    # Le bord d'une table n'est jamais plus près du bassin : pieds contre la
    # table, pas plus. La pointe des pieds du G1 debout est à 9,9 cm devant
    # le bassin (URDF : sphère de contact de la pointe, posture debout de la
    # marche ; test_equilibre.py le vérifie). C'était 15 cm, 5 de marge ;
    # décision de revue : prendre ces 5 cm, sans plus. Puis, seconde
    # décision : compté du bassin au vrai bord (distance_ideale), tant que
    # les avant-bras passent à marge_plateau_m au-dessus du plateau.
    bord_min_m: float = 0.10
    # Erreur d'arrivée de la marche à la pose de prise (celle que tire la
    # campagne de cartons variés) : les pieds n'en restent pas moins derrière
    # le bord. Constaté sur cette campagne, pieds contre la table : 5
    # approches sur 94 mettaient un pied 0,7 à 1,8 cm au-delà du bord.
    erreur_approche_m: float = 0.02
    erreur_cap_rad: float = math.radians(3.0)
    duree_placement_s: float = 0.8
    vitesse_fermeture: float = 0.5     # rad/s, roulis d'épaule
    vitesse_contact: float = 0.2       # rad/s, face commandée à moins de `zone_contact_m`
    zone_contact_m: float = 0.015      # de la face attendue : petit pas, petit dépassement
    pas_contact_max_rad: float = 0.004  # et jamais plus par pas (un pas lent ne l'agrandit pas)
    hausse_contact_nm: float = 1.5     # hausse du couple qui signe le contact
    retard_contact_rad: float = 0.06   # ou le roulis ne suit plus sa consigne
    # ou, en géométrie (constaté sur GPU le 2026-10-29 : le roulis suit sa
    # consigne, ce sont le poignet et le coude qui cèdent), la face
    # intérieure MESURÉE de la main en retard d'au moins `retard_face_m` sur
    # sa face COMMANDÉE, le long de la normale de l'objet, et à moins de
    # `tol_face_contact_m` de la face attendue de l'objet.
    retard_face_m: float = 0.01
    tol_face_contact_m: float = 0.015
    # Borne de la consigne de fermeture : la face COMMANDÉE de chaque main
    # ne va jamais au-delà de largeur/2 - penetration_max_m du centre.
    penetration_max_m: float = 0.02
    # Vide : les faces intérieures des mains (cinématique sur TOUS les
    # angles mesurés) plus proches que la largeur de l'objet moins vide_m.
    # Jamais d'après le seul angle du roulis : bras affaissés, ce même
    # angle ne ferme pas autant (constaté sur GPU, 2026-10-29).
    vide_m: float = 0.02
    ecart_largeur_max_m: float = 0.05  # contact bien avant la largeur attendue : autre chose
    # Force estimée à la main (N) : pseudo-inverse amortie de la transposée
    # de la jacobienne (`amorti_force_m`), appliquée à la hausse des couples
    # de TOUTES les articulations du bras depuis la fermeture, poids des bras
    # retiré (URDF), composante le long de la normale de l'objet, moyenne
    # glissante sur `filtre_force` lectures. Bruit : les couples estimés du
    # G1 (bruit_couples.py, 0,3 N·m par lecture) donnent ~0,85 N par lecture
    # à la main, ~0,4 N filtrés ; le bruit réel est mesuré au repos avant
    # chaque fermeture (`duree_bruit_s`), et le seuil de contact n'est jamais
    # sous `sigmas_contact` fois ce bruit filtré.
    amorti_force_m: float = 0.02
    filtre_force: int = 5
    duree_bruit_s: float = 0.2
    # Le zéro des couples (τ0) n'est pris qu'à `duree_raidissement_s` au
    # moins du dernier changement de gains. Pourquoi (analyse de l'essai GPU
    # T1 de novembre 2026, force estimée de mauvais signe pendant toute la
    # fermeture, contacts réels manqués) : les gains étaient raidis au début
    # même de la fenêtre du zéro ; le bras, moins affaissé sous son poids,
    # bougeait pendant cette fenêtre, et le couple de ce mouvement, pris dans
    # τ0, décalait la force relative jusqu'au bout. Les gains sont désormais
    # raidis dès le placement des mains.
    duree_raidissement_s: float = 0.5
    # Référence du contrôle géométrique d'appui (`retard_appui_m`, plus bas) :
    # l'écart de chaque main à sa consigne, mains libres, pris au début du
    # placement, une fois les bras posés (raidis depuis
    # `duree_raidissement_s` au moins, et cet écart stable à
    # `stabilite_pose_m` près pendant `duree_pose_s`) ; le geste ne commence
    # qu'ensuite. Avant, il était pris à l'appel de prendre(), bras parfois
    # encore en mouvement : « main déjà en appui (retenue à 1.6 cm) » sans
    # aucun contact (CI de la PR 24, machine chargée). Bras qui ne se posent
    # pas en `pose_max_s` : prise refusée.
    duree_pose_s: float = 0.2
    stabilite_pose_m: float = 0.001
    pose_max_s: float = 3.0
    force_contact_n: float = 3.0
    sigmas_contact: float = 6.0
    # Serrage visé et minimal, à chaque main. Bras du G1 aux gains du banc
    # T1 (raidis) : ~500 N/m à la main, soit ~10 N au plus sur un objet
    # raide pour les 2 cm de `penetration_max_m` ; un carton de 1 kg entre
    # deux mains caoutchouc (frottement ~1) demande 4,9 N à chacune.
    force_serrage_n: float = 8.0
    force_min_n: float = 5.0
    # Couples estimés NON FIABLES (CalibrationCouples.fiabilite : ils ne
    # suivent pas la pesanteur du modèle), la force à la main ne décide plus
    # rien : le serrage se juge en géométrie. La face commandée avance
    # jusqu'à sa borne (`penetration_max_m`), et chaque main doit y rester
    # RETENUE par l'objet : face mesurée en retard d'au moins
    # `retard_serrage_m` sur la commandée (depuis le début de la fermeture,
    # comme le contact), à moins de `tol_face_contact_m` de la face touchée,
    # et la main bloquée (sa face mesurée immobile à `glissement_serrage_m`
    # près) pendant `duree_maintien_s`, consigne fixe. 1 cm : ~5 N (`force_min_n`) à ~500 N/m, la raideur
    # à la main des gains raidis du banc T1 ; constaté sur GPU (essai T1 du
    # 5 novembre 2026), 1.6 à 1.9 cm de retard pour 6 à 9.5 N réels.
    retard_serrage_m: float = 0.01
    glissement_serrage_m: float = 0.003
    force_deja_n: float = 16.0         # à la prise, avant de bouger : bras déjà en effort
    pas_serrage_max_m: float = 0.0005  # avance de la face commandée par pas, au plus
    pas_serrage_min_m: float = 0.0001
    duree_serrage_max_s: float = 6.0   # au-delà, on juge la force obtenue
    # Recalage au toucher : les faces mesurées des deux mains au contact
    # disent où est vraiment le carton, le long de sa normale ; au-delà de
    # `recalage_max_m` de la perception (l'erreur de position acceptée), la
    # prise est refusée. Jamais plus que jeu_ouverture_m - marge_ouverture_m.
    recalage_max_m: float = 0.03
    # Filet avant de fermer : la force ABSOLUE à chaque main (poids du bras
    # retiré par le modèle, pas le zéro de la fermeture), au-delà de
    # max(force_appui_n, sigmas_appui × bruit mesuré au repos), veut dire une
    # main déjà en appui sur l'objet : prise refusée. Seuil provisoire, à
    # régler sur les essais GPU (valeur mesurée au journal).
    force_appui_n: float = 3.0
    sigmas_appui: float = 4.0
    # … et en géométrie : la main MESURÉE (cinématique sur les angles
    # mesurés) retenue en dehors de sa main COMMANDÉE (cinématique sur la
    # consigne), le long de la normale de l'objet, de `retard_appui_m` de
    # plus qu'au début du placement (mains grandes ouvertes, libres) : la
    # main est retenue par quelque chose à la pose d'ouverture.
    # L'affaissement des bras sous leur poids, et sa compensation, sont les
    # mêmes au départ et à l'arrivée : ils s'annulent.
    retard_appui_m: float = 0.01
    # Plafond de couple de CHAQUE articulation du bras (valeur absolue).
    effort_max_nm: float = 10.0        # jamais au-delà
    part_couple_garde: float = 0.5     # ni au-delà de cette part du couple max du garde-fou
    duree_maintien_s: float = 0.3
    age_max_effort_s: float = 0.10
    hauteur_test_m: float = 0.04       # levage de vérification
    # Levage lent et progressif (2 cm/s ; 5 cm/s jusqu'au 5 novembre 2026) :
    # le carton qui commence à tourner est vu avant d'être loin de la table.
    duree_levage_test_s: float = 2.0
    essais_max: int = 2
    distance_torse_m: float = 0.10     # face arrière de l'objet porté, devant le bassin
    hausse_transport_m: float = 0.05
    duree_ramener_s: float = 2.5
    inclinaison_max_deg: float = 15.0
    # Carton CONTRE le torse : un troisième appui, qui l'empêche de tourner
    # autour de la ligne des mains (essai GPU du 5 novembre 2026, 16 h 06 :
    # tenu à deux mains seulement, il a basculé puis glissé). On le ramène
    # jusqu'à `appui_torse_m` au-delà du buste attendu (`distance_torse_m`),
    # en s'arrêtant au toucher : les mains tirent de `force_torse_n` vers le
    # robot (couples fiables), ou restent en retard de `retard_torse_m` sur
    # leur consigne. 2 cm au-delà au plus : à 4, la consigne des mains
    # s'enfonçait de 2,9 cm dans le carton en chemin (2 + 0,2 au plus).
    appui_torse_m: float = 0.02
    force_torse_n: float = 3.0
    retard_torse_m: float = 0.01
    # Basculement autour de la ligne des mains. Les mains caoutchouc touchent
    # presque en un point : le couple qu'elles retiennent autour de la ligne
    # qui les joint n'est que leur frottement en rotation, 2 × (2/3) μ N r
    # (deux disques de contact de rayon r, pression uniforme ; couple_retenu).
    # Le poids, à e de cette ligne en profondeur, fait basculer de m g e ;
    # à h au-dessus, il ne fait tourner qu'une fois le carton penché de θ,
    # de m g h sin θ. D'où les tolérances de la ligne des mains autour du
    # centre de gravité (tolerances_cg) : e, h ≤ couple retenu / (sécurité ×
    # m g [× sin θ]), soit, aux valeurs ci-dessous (frottement statique du
    # carton du banc ; caoutchouc écrasé sur ~2 cm de rayon sous 8 N ; 25 %
    # de marge), 1,45 cm en profondeur et 8,3 cm en hauteur (1,7 et 10 pour
    # 1 kg). Essai GPU du 5 novembre 2026 : 11 cm en profondeur, 6 fois la
    # tolérance. Valeurs à confirmer sur GPU (frottement du caoutchouc,
    # rayon du contact).
    frottement_main: float = 1.0
    rayon_contact_main_m: float = 0.02
    # Prise en pendule : la ligne des mains AU-DESSUS du centre de gravité.
    # Essai GPU du 20 novembre 2026, 21 h 37 : ligne 0,2 à 1 cm au-dessus du
    # centre (l'OS la croyait 1,9 cm dessous), centre décalé de 0,9 cm en
    # profondeur (l'OS : 0) ; le frottement en rotation du caoutchouc ne
    # retenait presque rien (rayon effectif sous 5 mm) : le carton a tourné de
    # 50°, l'équilibre d'un pendule presque sans bras (tan θ = e / d). Sous la
    # ligne d'au moins d, le poids le ramène (couple de rappel m g d sin θ) ;
    # au-dessus, rien ne le ramène. Les accélérations de la marche (~1 m/s²)
    # le balancent de atan(a / g) ≈ 6° au plus, quel que soit d, et il revient
    # (essai de 22 h 04 : ±3° au pas) ; un décalage en profondeur e non vu
    # le penche de atan(e / d).
    #   - marge_pendule_m : au moins, la ligne au-dessus du centre, estimée par
    #     l'OS au serrage (mains mesurées) : plus que l'erreur de l'OS sur la
    #     hauteur de la ligne à 21 h 37 (2,4 cm : vue à -1,9, vraie à +0,5),
    #     pour que le centre soit sous la ligne à coup sûr ;
    #   - affaissement_prise_m : de combien les mains se posent plus bas que
    #     visé (bras sous leur poids, fermeture) : 1,9 cm à 21 h 37, 2,6 cm
    #     sur le robot simulé ; la visée est marge + affaissement ;
    #   - bord_face_m : le contact à au moins 3 cm sous l'arête du dessus (la
    #     paume a une étendue, BORD_FACE_M du robot simulé : 2 cm, plus 1 cm) ;
    #     un carton trop bas pour la visée (moins de 18 cm) est refusé, avec
    #     la raison.
    #   Un décalage en profondeur de 0,9 cm (21 h 37) penche alors le carton
    #   de atan(0,9 / 3) = 17° au pire, sans aucun frottement en rotation, de
    #   8,5° visé à 6 cm (la caméra l'arrête à rotation_vue_max_deg).
    marge_pendule_m: float = 0.03
    affaissement_prise_m: float = 0.03
    bord_face_m: float = 0.03
    # La charge la plus lourde que l'OS porte : les tolérances ci-dessus sont
    # calculées pour elle, et un carton pesé au-delà est reposé aussitôt, avant
    # de serrer plus et de le porter. 1 kg, la masse du carton T1, était réglé
    # sur lui seul ; la campagne de cartons variés (axis_os/testing/cartons.py)
    # montre que les mains du G1 portent jusqu'à ~1,2 kg sous leurs plafonds.
    masse_max_kg: float = 1.2
    securite_basculement: float = 1.25
    # Le carton tenu qui tourne de plus de `rotation_max_deg` autour de la
    # ligne des mains (caméra de tête quand elle le voit), ou qui ne pèse
    # plus sur les mains pendant le port (moins de `part_poids_min` du poids
    # du début du port, pendant `duree_perte_s`) : on le repose au lieu de
    # marcher avec (ou, en marche, on s'arrête et on le signale).
    rotation_max_deg: float = 10.0
    # Vue par la caméra de tête (en entier, ou son dessus en partie), le
    # carton tenu s'arrête plus tôt : à rotation_vue_max_deg. Il tourne
    # jusqu'à ~8°/s entre les paumes (essai GPU du 20 novembre 2026, 21 h 37 :
    # de -4,8° à -8,7° en 0,5 s au début du port) ; 2° d'erreur de la vue et
    # une image de retard (0,1 s) ne le laissent pas passer
    # rotation_max_deg. Ne sert qu'à arrêter et reposer,
    # jamais à autoriser : un carton vu droit n'efface aucune autre alerte.
    rotation_vue_max_deg: float = 6.0
    part_poids_min: float = 0.4
    # Poids qui chute : jugé sur les filtre_force dernières lectures (0,1 s),
    # pendant duree_perte_s, pour qu'un carton qui glisse ou s'échappe soit vu
    # en 0,3 s au plus (avant : moyenne de 0,3 s, puis 0,3 s, ~0,5 s en tout).
    duree_perte_s: float = 0.15
    # Serrage d'après le poids (essai GPU T1 du 5 novembre 2026, 21 h 57 :
    # carton de 10,9 N tenu à 6,5 N par main, soit un frottement d'au moins
    # 0,84 rien que pour le porter, aucune marge ; il a glissé à la marche).
    # En haut du levage, la prise confirmée, l'OS pèse le carton une fois,
    # bras immobiles, avant de serrer plus (lectures_pesee, plus bas). Ce
    # poids fixe le serrage requis par main :
    # securite_serrage × poids / (2 frottement_serrage) ; l'OS vise
    # marge_serrage_n de plus, et se
    # contente du requis s'il ne peut aller au-delà (au-dessus du serrage
    # déjà atteint seulement : un objet léger garde celui d'avant).
    # frottement_serrage : hypothèse prudente
    # (caoutchouc sur carton ; 1,0 au banc). marge_serrage_n : à la marche, la
    # charge passe d'une main à l'autre, la somme des deux forces normales
    # varie de ~1 N (juge de l'essai). La face commandée peut alors entrer
    # jusqu'à penetration_poids_max_m dans la face touchée (raideur ~310 N/m
    # au point de contact sur GPU : 15 N demandent ~5 cm), plafonds de couple
    # inchangés ; le levage attend en haut ce serrage. Aucune main ne peut
    # plus avancer (borne atteinte, ou une articulation à moins de 0,3 N·m de
    # son plafond) pendant duree_serrage_poids_max_s : le carton est reposé
    # (trop lourd pour ces mains). Si les faces mesurées se rapprochent de
    # plus de cede_max_m, le carton cède sous le serrage : reposé aussi.
    frottement_serrage: float = 0.8
    securite_serrage: float = 2.0
    # Pesé sous poids_min_n (~100 g), le carton n'est pas porté : resté sur la
    # table, les mains ont glissé le long de ses faces. Campagne de cartons
    # variés (axis_os/testing/cartons.py) : un carton de 1,8 kg à peine tenu
    # par le serrage de prise y était resté, mains 3,8 cm plus haut ; l'OS
    # confirmait la prise, pesait 0,2 N et partait avec. Prise non confirmée.
    poids_min_n: float = 1.0
    # La pesée : `lectures_pesee` lectures prises bras immobiles (aucune
    # articulation des bras mesurée à plus de `vitesse_pesee_max_rad_s` d'une
    # lecture à l'autre ; les autres sont écartées), leur moyenne tronquée
    # (poids_pese : `part_ecartee_pesee` des plus basses et autant des plus
    # hautes écartées). Essai GPU T1 du 19 novembre 2026, 22 h 04 : 10,6 N
    # pesés pour 9,8 sur 15 lectures (0,3 s), quand une lecture a ~1 N d'écart
    # type (bruit des couples estimés, 0,3 N·m par articulation) : 15 lectures
    # ne sont sûres qu'à ±3 % (un écart type) ; les couples du relevé, aux
    # mêmes instants, donnaient 9,65 N. 100 lectures (2 s, décision sur la PR
    # #38) : ±2 % à 95 %, celles de l'essai tirées avec remise ; 50 en
    # laissaient ±2,8 %, quand le carton de 1 kg, poignets à leurs plafonds, ne
    # se porte qu'avec un poids juste à ~2 % près. Pas assez de
    # lectures immobiles en duree_pesee_max_s : pesé sur ce qu'on a s'il y en a
    # au moins filtre_force × 3 (la fenêtre d'avant), sinon reposé ; plus de
    # lectures écartées qu'immobiles (au moins filtre_force × 3) : le carton
    # bouge entre les mains, décidé sans attendre. Une main
    # qui passe sous force_min_n écourte la pesée (au moins filtre_force × 3
    # lectures) : on ne reste pas au serrage de prise pendant qu'elle se
    # relâche (carton pris 2 cm à côté de sa perception : la main la moins
    # enfoncée passait de 6,8 à 3,6 N en 1 s).
    lectures_pesee: int = 100
    vitesse_pesee_max_rad_s: float = 0.05
    part_ecartee_pesee: float = 0.1
    duree_pesee_max_s: float = 3.0
    # Calibration de charge (une fois par robot, à la première mise en route :
    # ManipulationService.calibrer_charge, docs/PREMIERE_MISE_EN_ROUTE.md) :
    # une masse de référence connue (au moins masse_reference_min_kg, au plus
    # masse_max_kg), prise comme un carton (la prise de T1), levée et pesée
    # comme lui, bras immobiles, mais plus longtemps
    # (lectures_calibration_charge lectures, au plus
    # duree_calibration_charge_max_s, au moins lectures_pesee) ; son poids,
    # partagé moitié-moitié entre les deux mains (charge centrée, prise par
    # ses deux faces), donne le gain du couple estimé de chaque coude
    # (gain_coude). Essais GPU T1 du 19 novembre 2026 : les coudes font
    # presque tout le biais de la pesée (+2 à +5 %) ; sur un palier de
    # 35 lectures, ce calcul retrouve leurs gains au banc (1,048 pour 1,047 ;
    # 0,981 pour 0,980). Hors de [gain_coude_min, gain_coude_max] :
    # calibration refusée (masse décentrée ou mal déclarée), rien n'est changé.
    lectures_calibration_charge: int = 250
    duree_calibration_charge_max_s: float = 8.0
    masse_reference_min_kg: float = 0.5
    gain_coude_min: float = 0.85
    gain_coude_max: float = 1.15
    marge_serrage_n: float = 1.0
    penetration_poids_max_m: float = 0.06
    duree_serrage_poids_max_s: float = 3.0
    cede_max_m: float = 0.01
    # Au port, les mains gardent leur serrage : la consigne desserrée par un
    # plafond est rendue dès que les bras le permettent ; si les mains
    # mesurées s'écartent de plus de ecart_face_port_m de leur place au
    # serrage (somme des deux mains : le balancement de la marche, ±2,5 cm
    # sur GPU, les décale ensemble et s'y annule ; la rotation de la taille
    # aussi), ou si la marge de frottement μ (N_G + N_D) / poids passe sous
    # securite_serrage, la base s'arrête et l'on resserre ; si cela ne suffit
    # pas en duree_resserrage_s, on s'arrête pour de bon et on le signale
    # (à reposer).
    ecart_face_port_m: float = 0.005
    duree_resserrage_s: float = 1.0
    # Mains écartées à répétition : un carton qui tourne entre les paumes
    # (essai GPU T1 du 20 novembre 2026, 21 h 37 : tourné de 50° autour de la
    # ligne des mains, paumes immobiles à 3,8° près ; seul signe vu par l'OS,
    # « mains écartées de 5 à 13 mm », 8 fois, chaque fois resserrées). Au
    # ecartements_max-ième écartement en fenetre_ecartements_s, ou dès que
    # l'écart grandit de croissance_ecart_m malgré le resserrage : transport
    # interrompu, à reposer, avec la raison. L'écart de référence est le plus
    # grand vu pendant delai_croissance_s (le temps que le resserrage agisse :
    # la main glisse encore quelques pas, robot simulé).
    ecartements_max: int = 2
    fenetre_ecartements_s: float = 10.0
    # Hystérésis, comme celle de la marge (arrêt sous 2,0, reprise au-dessus de
    # 2,2) : arrêtée pour un écartement, la base ne repart que l'écart revenu
    # sous ecart_face_port_m - hysteresis_ecart_m (le resserrage l'y amène) ;
    # alors seulement l'écartement finit, et le suivant peut compter. Le
    # bruit de l'écart lu, en marchant : 0,2 à 0,45 mm par lecture (écart-type
    # autour de sa médiane sur 0,3 à 0,5 s, hors l'arrondi au mm du journal ;
    # essais GPU 17 h 16, 18 h 21, 21 h 37). Un vrai écartement, puis ce bruit
    # 10 s (boucle simulée, resserrage de 0,25 à 1 mm par lecture) : 2e
    # écartement compté à tort, donc transport interrompu, 59 à 100 % du temps
    # sans hystérésis, 10 à 87 % à 1 mm, 0 à 8,5 % à 2 mm, 0,1 % au plus à 3.
    hysteresis_ecart_m: float = 0.003
    croissance_ecart_m: float = 0.002
    delai_croissance_s: float = 0.2
    # Au retour (RAMENER), les deux mains bougent comme un seul corps rigide
    # avec le carton (essai GPU T1 du 6 novembre 2026, 18 h 21 : la
    # cinématique inverse ne tenait que la POSITION des points de contact ;
    # les deux mains ont piqué de 18° autour de la ligne des mains, la gauche
    # a roulé de 10° de plus que la droite, et le carton s'est tordu entre
    # elles : 9,7° de rotation, 4,6 cm de glissement à la main gauche, 2 à
    # 3 N de serrage perdus). Chaque main garde son orientation, à
    # tol_rigide_deg près, et suit la même translation, à tol_rigide_m près,
    # sur tout le chemin vérifié d'avance (étapes de pas_chemin_rigide_m,
    # aucune articulation qui saute de plus de saut_rigide_rad d'une étape à
    # l'autre, avant-bras au-dessus du plateau) ; sinon on rapproche moins,
    # ou pas du tout. Pendant le retour rigide, le roulis (autour de l'axe avant) et
    # le tangage (autour de la ligne des mains) du carton, estimés sur les
    # mains mesurées (ligne des mains, orientation des paumes), ne passent
    # pas rotation_ramener_max_deg : au-delà, le geste s'arrête et l'on
    # resserre.
    tol_rigide_m: float = 0.001
    tol_rigide_deg: float = 1.0
    pas_chemin_rigide_m: float = 0.01
    saut_rigide_rad: float = 0.15
    rotation_ramener_max_deg: float = 4.0
    # Pendant TOUT le port (essai GPU T1 du 19 novembre 2026, 17 h 16 : le
    # carton a tourné de 24° pendant 20 s de marche, aucune alerte de l'OS ;
    # le resserrage poussait une main pendant que le plafond du poignet
    # droit faisait reculer l'autre, la paire a glissé de 2,2 cm sans serrer
    # plus, 32 arrêts de la base sur le seuil de la marge) :
    #   - la pose des deux mains est figée en corps rigide dans le repère du
    #     torse au début du port ; le serrage et le desserrage n'y bougent les
    #     mains que le long de la normale du carton, symétriquement, les bras
    #     recalculés depuis la posture figée (rien d'autre ne change) ;
    #   - les mains MESURÉES (cinématique, repère du torse) sont comparées à
    #     leur consigne, moyenne glissante de fenetre_derive_s (le balancement
    #     de la marche s'y annule) : une dérive commune de plus de
    #     derive_port_m ou derive_port_deg depuis le début du port est
    #     corrigée vers la pose figée, en tout au plus correction_port_max_m
    #     et correction_port_max_deg ;
    #   - roulis et tangage de la ligne des mains, lacet de la paire
    #     (paumes mesurées, depuis le début du port) : au-delà de
    #     rotation_ramener_max_deg, base arrêtée et correction ; au-delà de
    #     rotation_max_deg, à reposer (la façade s'arrête et le signale) ;
    #   - hystérésis du resserrage : la base s'arrête sous securite_serrage,
    #     ne repart qu'au-dessus de securite_reprise (jamais moins que
    #     securite_serrage), et pas avant delai_reprise_s ; sans reprise en
    #     duree_resserrage_s, à reposer.
    derive_port_m: float = 0.01
    derive_port_deg: float = 2.0
    fenetre_derive_s: float = 1.0
    correction_port_max_m: float = 0.03
    correction_port_max_deg: float = 6.0
    securite_reprise: float = 2.2
    delai_reprise_s: float = 0.5
    # La marge de frottement jugée sur une foulée (fenetre_marge_s, dès
    # filtre_force × 3 lectures au début du port) : le serrage réel varie au
    # pas de la marche (essai GPU T1 du 19 novembre 2026, 22 h 04 : juge de
    # 10,5 à 14,3 N par main), la marge sur 0,3 s le suivait et arrêtait la
    # base au ras de securite_serrage, le carton tenu (marge réelle sur 1 s de
    # 2,03 à 2,12). Le glissement, lui, reste surveillé directement, à 0,3 s au
    # plus : mains écartées et rotation à chaque pas, poids qui chute
    # (duree_perte_s), et la marge sur les filtre_force × 3 dernières lectures
    # (0,3 s), qui ne doit jamais passer sous securite_glissement (1 : le
    # serrage ne porte plus le carton au frottement prudent ; la marche, elle,
    # ne la fait varier que de quelques centièmes) : un vrai desserrage arrête
    # la base aussitôt, sans attendre la foulée. Base arrêtée pour resserrer, la
    # reprise se juge sur 0,3 s (la marche ne fait plus varier le serrage, et la
    # foulée ne verrait l'effet du resserrage qu'au bout de 1 s, l'échéance de
    # duree_resserrage_s) ; à la reprise, la foulée repart de ces 0,3 s.
    fenetre_marge_s: float = 1.0
    securite_glissement: float = 1.0
    # Les bras à leurs plafonds (une articulation à moins de 0,3 N·m du sien,
    # ou une main à sa borne) ne peuvent serrer plus : securite_reprise est
    # hors de leur portée. Vrai : la base repart dès que la marge tient
    # securite_serrage, jamais en dessous (comme avant l'hystérésis). Faux :
    # elle ne repart jamais sous securite_reprise ; sans reprise en
    # duree_resserrage_s, à reposer.
    reprise_aux_plafonds: bool = True
    # Écart admis entre une main mesurée par le robot (le lien de la main,
    # rubber_hand) et la même main calculée par la cinématique sur les
    # angles mesurés. Au-delà, le modèle des bras ne décrit pas le robot :
    # pas de prise mesurée (constaté sur GPU le 2026-10-28 : mains contre
    # la face avant du carton, ~10 cm du milieu des faces).
    ecart_modele_max_m: float = 0.015

    def __post_init__(self):
        # Refus au démarrage : une ouverture qui ne couvre pas l'erreur de
        # position acceptée, marge comprise, ferait fermer une main déjà en
        # appui sur l'objet.
        if self.recalage_max_m > self.jeu_ouverture_m - self.marge_ouverture_m + 1e-12:
            raise ValueError(
                f"réglages de prise refusés : erreur de position acceptée (recalage_max_m = "
                f"{self.recalage_max_m:.3f} m) au-delà de l'ouverture moins sa marge "
                f"(jeu_ouverture_m - marge_ouverture_m = "
                f"{self.jeu_ouverture_m - self.marge_ouverture_m:.3f} m)")

    def avance_max(self, profondeur: float) -> float:
        """Écart maximal (m) des mains vers la face avant, depuis le milieu
        de l'objet : `recul_face_min_m` derrière la face avant au moins, et
        jamais plus que `tol_profondeur_m`. Un objet de moins de deux fois
        `recul_face_min_m` de profondeur est pris au milieu, à
        `tol_profondeur_m` près (la règle ne peut pas y tenir)."""
        reste = profondeur / 2.0 - self.recul_face_min_m
        return self.tol_profondeur_m if reste <= 0.0 else min(self.tol_profondeur_m, reste)


@dataclass
class MesuresBras:
    """Ce que le robot mesure, et que la prise mesurée exige.

    modele      : la cinématique des bras (URDF).
    positions   : () -> angles bruts mesurés des articulations des bras.
    efforts     : () -> {articulation: (couple en N·m, âge de la mesure en s)}.
    bassin_z    : () -> hauteur du bassin, repère monde (m).
    couple_max  : couple maximal du garde-fou par articulation (N·m).
    inclinaison : () -> inclinaison de l'objet tenu (degrés), ou None si
                  elle n'est pas mesurée.
    vers_bassin : point du repère monde -> repère du bassin, rotation
                  complète (tangage et roulis du bassin compris), celui de
                  la cinématique. None : le cap seul, et la hauteur au-dessus
                  du bassin (bassin supposé droit).
    raideur     : (bool) -> None ; True raidit les poignets (et le coude)
                  par leurs gains kp/kd, dans les limites du pilote, False
                  rend les gains normaux. Appelé à la fermeture, puis au
                  dépôt, à l'échec et sur arrêt d'urgence. None : pas de
                  réglage de gains.
    arret_urgence : () -> arrêt d'urgence en cours ? (gains rendus aussitôt)
    commandables : articulations des bras que la prise peut commander (en
                  plus du tangage, du roulis et du coude : lacet d'épaule,
                  poignets) pour la fermeture et le serrage en cartésien.
                  None : tangage, roulis et coude seulement.
    rotation    : () -> rotation de l'objet tenu autour de la ligne des mains
                  (degrés, depuis la prise), vue par la caméra de tête ; None
                  quand elle ne le voit pas. None : pas de caméra pour cela.
    contact_main_m : où les mains de CE robot touchent, le long de la main,
                  depuis son repère (voir ReglagesPrise.contact_main_m, la
                  valeur du G1, prise si None).
    """
    modele: ModeleBras
    positions: Callable[[], Dict[str, float]]
    efforts: Callable[[], Dict[str, Tuple[float, float]]]
    bassin_z: Callable[[], float]
    couple_max: Dict[str, float] = field(default_factory=dict)
    inclinaison: Optional[Callable[[], Optional[float]]] = None
    vers_bassin: Optional[Callable[[np.ndarray], np.ndarray]] = None
    raideur: Optional[Callable[[bool], None]] = None
    arret_urgence: Optional[Callable[[], bool]] = None
    commandables: Optional[Tuple[str, ...]] = None
    rotation: Optional[Callable[[], Optional[float]]] = None
    contact_main_m: Optional[float] = None


def efforts_valides(mesures: MesuresBras, joints: Iterable[str], age_max: float,
                    optionnels: Iterable[str] = ()) -> Tuple[Optional[Dict[str, float]], str]:
    """Couples des articulations `joints`, ou (None, raison) si une mesure
    manque, n'est pas finie ou est périmée : jamais « tout va bien ». Les
    `optionnels` sont ajoutés quand le robot les mesure (absents : ignorés,
    illisibles : refus)."""
    try:
        brut = mesures.efforts()
    except Exception as exc:              # noqa: BLE001 — capteur en panne
        return None, f"lecture de l'effort impossible ({type(exc).__name__})"
    requis = list(joints)
    sortie = {}
    for j in requis + [o for o in optionnels if o not in requis]:
        v = brut.get(j) if isinstance(brut, dict) else None
        if v is None:
            if j not in requis:
                continue
            return None, f"effort de {j} absent"
        try:
            couple, age = float(v[0]), float(v[1])
        except (TypeError, ValueError, IndexError):
            return None, f"effort de {j} illisible"
        if not (math.isfinite(couple) and math.isfinite(age)):
            return None, f"effort de {j} non fini"
        if age < 0.0 or age > age_max:
            return None, f"effort de {j} périmé ({age:.2f} s)"
        sortie[j] = couple
    return sortie, ""


def positions_valides(mesures: MesuresBras, joints: Iterable[str]
                      ) -> Tuple[Optional[Dict[str, float]], str]:
    try:
        brut = mesures.positions()
    except Exception as exc:              # noqa: BLE001
        return None, f"lecture des angles impossible ({type(exc).__name__})"
    sortie = {}
    for j in joints:
        v = brut.get(j) if isinstance(brut, dict) else None
        if v is None or not math.isfinite(float(v)):
            return None, f"angle de {j} absent ou non fini"
        sortie[j] = float(v)
    return sortie, ""


def faces(centre: np.ndarray, largeur: float, lacet: float
          ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Milieux des faces gauche et droite de l'objet (repère du bassin), et
    la normale de la face gauche ; `lacet` : rotation de l'objet par
    rapport au robot (rad)."""
    n = np.array([-math.sin(lacet), math.cos(lacet), 0.0])
    return centre + n * largeur / 2.0, centre - n * largeur / 2.0, n


def cibles_mains(centre: np.ndarray, largeur: float, lacet: float, ecart: float
                 ) -> Tuple[np.ndarray, np.ndarray]:
    """Mains à `ecart` de chaque face, vers l'extérieur (les « mains » du
    modèle : leur point de contact avec un modèle au_contact())."""
    fg, fd, n = faces(centre, largeur, lacet)
    return fg + n * ecart, fd - n * ecart


def couple_retenu(r: "ReglagesPrise", force_n: float) -> float:
    """Couple (N·m) que les deux mains retiennent autour de la ligne qui
    les joint, serrées de `force_n` chacune : leur frottement en rotation
    (voir ReglagesPrise, basculement)."""
    return 2.0 * (2.0 / 3.0) * r.frottement_main * max(0.0, force_n) * r.rayon_contact_main_m


def poids_pese(lectures: Iterable[float], part_ecartee: float) -> Tuple[float, float]:
    """Poids (N) pesé sur `lectures` : leur moyenne tronquée (`part_ecartee`
    des plus basses et autant des plus hautes écartées ; une lecture
    aberrante, comme la première après la remise à zéro de la moyenne
    glissante, n'y pèse pas), et son incertitude (N, un écart type :
    écart type des lectures sur leur racine carrée)."""
    x = np.sort(np.asarray(list(lectures), dtype=float))
    k = int(part_ecartee * len(x))
    garde = x[k:len(x) - k] if len(x) > 2 * k else x
    return float(np.mean(garde)), float(np.std(x) / math.sqrt(len(x)))


def gain_coude(poids_main_n: float, part_coude_n: float, poids_n: float) -> float:
    """Gain du couple estimé du coude d'un bras (estimé = gain × vrai),
    d'après la pesée d'une masse connue de `poids_n` (N) partagée
    moitié-moitié entre les deux mains : la force vers le haut estimée à
    cette main vaut `poids_main_n`, dont `part_coude_n` vient du couple du
    coude (ce qu'elle perd sans lui). Corrigé du gain k, ce couple donne
    poids_main_n + (1/k - 1) part_coude_n, qui doit valoir poids_n / 2.
    NaN si le coude ne porte rien (pas de calibration possible)."""
    if not part_coude_n > 1e-6:
        return math.nan
    u = 1.0 + (poids_n / 2.0 - poids_main_n) / part_coude_n
    return 1.0 / u if u > 1e-6 else math.nan


def tolerances_cg(r: "ReglagesPrise", force_n: Optional[float] = None,
                  masse_kg: Optional[float] = None) -> Tuple[float, float]:
    """Écarts admis (m) entre la ligne des mains et le centre de gravité de
    l'objet : (en profondeur, en hauteur), serré à `force_n` (par défaut
    force_serrage_n), pour un objet de `masse_kg` (par défaut masse_max_kg)."""
    poids = 9.81 * (r.masse_max_kg if masse_kg is None else masse_kg)
    retenu = couple_retenu(r, r.force_serrage_n if force_n is None else force_n)
    e = retenu / (r.securite_basculement * poids)
    return e, e / math.sin(math.radians(r.rotation_max_deg))


def hauteur_prise(r: "ReglagesPrise", hauteur_objet: float) -> Optional[float]:
    """De combien (m) viser les mains au-dessus du centre d'un objet de
    `hauteur_objet` (prise en pendule) : marge_pendule_m plus
    affaissement_prise_m (elles se posent plus bas que visé) ; None si cela
    les met à moins de bord_face_m de l'arête du dessus (objet trop bas pour
    une prise en pendule)."""
    dz = r.marge_pendule_m + r.affaissement_prise_m
    return dz if dz <= hauteur_objet / 2.0 - r.bord_face_m + 1e-9 else None


def ecart_ligne_cg(contact_g: np.ndarray, contact_d: np.ndarray, cg: np.ndarray
                   ) -> Tuple[float, float]:
    """Où passe la ligne qui joint les points de contact des mains, par
    rapport au centre de gravité `cg` (repère du bassin) : (en profondeur,
    en hauteur), en m, signés : la ligne en arrière du centre (loin du
    robot) et au-dessus de lui sont positifs."""
    g, d, c = (np.asarray(v, dtype=float) for v in (contact_g, contact_d, cg))
    a = d - g
    a = a / max(1e-9, float(np.linalg.norm(a)))
    p = g + float((c - g) @ a) * a              # point de la ligne le plus proche du centre
    v = p - c
    haut = np.array([0.0, 0.0, 1.0])
    prof = np.cross(haut, a)                    # horizontale, perpendiculaire à la ligne
    if prof[0] < 0:
        prof = -prof                            # vers l'avant du robot (loin de lui)
    return float(v @ (prof / max(1e-9, float(np.linalg.norm(prof))))), float(v @ haut)


def resoudre(modele: ModeleBras, cg: np.ndarray, cd: np.ndarray, depart: Dict[str, float],
             tol: float = 0.003) -> Tuple[Optional[Dict[str, float]], float]:
    """Angles des deux bras pour ces repères de mains (à `tol` m près), ou
    (None, erreur)."""
    qg, eg = modele.gauche.inverse(cg, depart, tol=tol)
    qd, ed = modele.droite.inverse(cd, depart, tol=tol)
    if qg is None or qd is None:
        return None, max(eg, ed)
    q = dict(qg)
    q.update(qd)
    if not (modele.gauche.dans_les_bornes(q) and modele.droite.dans_les_bornes(q)):
        return None, max(eg, ed)
    return q, max(eg, ed)


def garde_plateau(modele: ModeleBras, q: Dict[str, float], plateau_z: float, bord_x: float,
                  rayon: float, cap: float = 0.0) -> float:
    """Distance (m) du dessous de l'avant-bras au plateau, sur la partie
    des avant-bras au-dessus de la table : au-delà de `bord_x` le long de la
    normale du bord, qui fait l'angle `cap` (rad) avec l'avant du robot (0 :
    face à la table, x ≥ bord_x) ; +inf si aucune."""
    n = (math.cos(cap), math.sin(cap))
    garde = math.inf
    for chaine in (modele.gauche, modele.droite):
        for p in chaine.avant_bras(q):
            if p[0] * n[0] + p[1] * n[1] >= bord_x:
                garde = min(garde, float(p[2]) - rayon - plateau_z)
    return garde


def distance_ideale(modele: ModeleBras, largeur: float, profondeur: float, hauteur: float,
                    centre_z: float, depart: Dict[str, float], *, bord_m: float = 0.0,
                    reglages: ReglagesPrise = ReglagesPrise(),
                    distances=np.arange(0.20, 0.45, 0.005),
                    seulement_au_vrai_bord: bool = False, cap_bord: float = 0.0,
                    debord_pieds: float = 0.0, marge_pieds: float = 0.0
                    ) -> Tuple[Optional[float], str]:
    """Distance (m) entre le bassin et le centre de l'objet, le long de
    l'axe du robot, pour que les mains atteignent le milieu de ses faces et
    que les avant-bras ne touchent pas le plateau. `centre_z` : hauteur du
    centre, repère du bassin ; `bord_m` : distance du centre de l'objet au
    bord de la table, moins sa demi-profondeur (de face : de sa face avant au
    bord) ; `distances` : des REPÈRES des mains au centre de l'objet,
    essayées dans cet ordre. (None, raison) s'il n'y en a aucune.

    Robot face à une face de l'objet, le bord de la table vu en biais :
    `cap_bord`, angle (rad) de la normale du bord avec l'avant du robot ;
    `debord_pieds`, de combien ses pieds avancent alors de plus vers le bord
    (avancee_pieds), ajouté à bord_min_m. 0 et 0 : face à la table.
    `marge_pieds` : de plus, au VRAI bord, pour les pieds seuls (erreur
    d'approche de la marche, en position et en cap) ; au premier palier, le
    bord des repères, 11 cm plus près, la couvre déjà.

    Les points de contact au milieu des faces, avec les bras dans la posture
    où leurs REPÈRES y seraient : celle de la distance trouvée pour les
    repères, l'objet `contact_main_m` plus loin. Plus près, la main ne pointe
    plus vers l'objet (coude replié en arrière) et le roulis ne ferme
    presque plus (constaté sur le robot simulé).

    1. Comme toujours : la plus courte qui convient (le carton reste près du
       corps), le bord de la table jugé depuis les repères, `contact_main_m`
       plus près d'eux qu'en vrai (plus sévère : le bassin reste à
       bord_min_m + contact_main_m du vrai bord au moins).
    2. Sinon, se rapprocher (décision de revue : « ce qui reste des 11 cm,
       tant que la garde des avant-bras est tenue ») : règle du bord et garde
       jugées au vrai bord, dans cette posture ET dans celle de la prise
       (ManipulationService._planifier : les points de contact au milieu des
       faces, la main comprise jusqu'à eux), pour que l'une ne contredise
       pas l'autre ; la plus courte qui convient, là encore.
    `seulement_au_vrai_bord` : le second palier seul."""
    r = reglages
    bout = r.contact_main_m
    au_contact = modele.au_contact(bout) if bout else None
    plateau = centre_z - hauteur / 2.0
    ca = math.cos(cap_bord)
    meilleure_garde = -math.inf
    paliers = (True,) if seulement_au_vrai_bord and bout else (False, True) if bout else (False,)
    for vrai_bord in paliers:
        for d in distances:
            # Du bassin au bord, le long de sa normale : celui de la règle
            # (repères ou vrai bord), et le vrai, pour les pieds.
            bord = (d + (bout if vrai_bord else 0.0)) * ca - profondeur / 2.0 - bord_m
            pieds = (d + bout) * ca - profondeur / 2.0 - bord_m
            if bord < r.bord_min_m + debord_pieds \
                    or pieds < r.bord_min_m + debord_pieds + marge_pieds:
                continue
            # Les mains visées au-dessus du centre (prise en pendule).
            c = np.array([float(d), 0.0, centre_z + (hauteur_prise(r, hauteur) or 0.0)])
            q, _ = resoudre(modele, *cibles_mains(c, largeur, 0.0, r.demi_main_m), depart)
            if q is None:
                continue
            g = garde_plateau(modele, {**depart, **q}, plateau, bord, r.rayon_avant_bras_m,
                              cap_bord)
            if vrai_bord:
                qc, _ = resoudre(au_contact, *cibles_mains(c + np.array([bout, 0.0, 0.0]),
                                                           largeur, 0.0, r.demi_main_m),
                                 {**depart, **q})
                if qc is None:
                    continue
                g = min(g, garde_plateau(au_contact, {**depart, **qc}, plateau, bord,
                                         r.rayon_avant_bras_m, cap_bord))
            meilleure_garde = max(meilleure_garde, g)
            if g >= r.marge_plateau_m:
                return float(d + bout), ""
    if meilleure_garde == -math.inf:
        return None, "objet hors de portée des bras à toute distance"
    return None, (f"avant-bras trop près du plateau à toute distance (au mieux "
                  f"{100 * meilleure_garde:.1f} cm, il faut {100 * r.marge_plateau_m:.0f})")


def ecarts_modele(modele: ModeleBras, angles: Dict[str, float],
                  mains_reelles: Tuple[np.ndarray, np.ndarray]
                  ) -> Tuple[np.ndarray, np.ndarray]:
    """Écarts (m, repère du bassin) entre les mains mesurées et les mains
    calculées par la cinématique sur les angles mesurés : (gauche, droite),
    réel moins modèle."""
    pg, pd = modele.mains(angles)
    return (np.asarray(mains_reelles[0], dtype=float) - pg,
            np.asarray(mains_reelles[1], dtype=float) - pd)


def signal_contact(r: "ReglagesPrise", hausse_nm: float, retard_roulis_rad: float,
                   force_n: float, seuil_force_n: float, retard_face_m: float,
                   ecart_face_m: float, couples_fiables: bool = True) -> Optional[str]:
    """Ce qui signe le contact d'une main en fermant, ou None : la hausse du
    couple du roulis, le roulis en retard sur sa consigne, la force à la main
    (au-delà du seuil mesuré au repos), ou la géométrie (face intérieure
    mesurée en retard de `retard_face_m` sur la commandée, à moins de
    `tol_face_contact_m` de la face attendue). Couples non fiables
    (CalibrationCouples.fiabilite) : ni couple, ni force."""
    if couples_fiables and hausse_nm >= r.hausse_contact_nm:
        return "couple"
    if retard_roulis_rad >= r.retard_contact_rad:
        return "retard du roulis"
    if couples_fiables and force_n >= seuil_force_n:
        return "force"
    if retard_face_m >= r.retard_face_m and ecart_face_m <= r.tol_face_contact_m:
        return "géométrie"
    return None


def main_retenue(r: "ReglagesPrise", retard_m: float, ecart_face_m: float) -> bool:
    """Serrage jugé en géométrie (couples non fiables) : la main est-elle
    retenue par l'objet ? Face mesurée en retard d'au moins
    `retard_serrage_m` sur la commandée, et à moins de `tol_face_contact_m`
    de la face touchée (pas un objet qui s'enfonce)."""
    return retard_m >= r.retard_serrage_m and ecart_face_m <= r.tol_face_contact_m


class CalibrationCouples:
    """Biais des couples estimés, articulation par articulation. Bras en
    l'air et immobiles, dans quelques postures (celles de la calibration des
    mains), le couple estimé devrait être celui de la pesanteur sur le bras
    (modèle de l'URDF, angles mesurés, centrale inertielle) ; l'écart moyen
    est le biais (moindres carrés d'un décalage constant sur tous les
    relevés), et ce qui reste, posture par posture, le résidu (le modèle
    n'explique pas tout). Constaté sur GPU (essai T1, novembre 2026) : 4.6 N
    puis 3.9 N à la main droite au repos, sans contact, pour un bruit de 0.6
    à 1.05 N : un biais, pas du bruit. La force à la main se calcule ensuite
    sur les couples corrigés (ManipulationService.biais_couples) ; les
    plafonds de couple, eux, restent jugés sur le couple mesuré."""

    def __init__(self):
        self.releves: list = []          # (posture, {articulation: (couple, gravité)})

    def ajouter(self, posture: str, couples: Dict[str, float],
                gravite: Dict[str, float]) -> None:
        self.releves.append((posture, {j: (float(couples[j]), float(gravite[j]))
                                       for j in gravite if j in couples}))

    def fiabilite(self, etendue_min_nm: float = 1.0, pente_min: float = 0.5,
                  pente_max: float = 1.5) -> Tuple[Optional[bool], Dict[str, float]]:
        """Les couples estimés suivent-ils la pesanteur du modèle ? Pour
        chaque articulation dont la pesanteur varie d'au moins
        `etendue_min_nm` d'une posture à l'autre : pente des couples moyens
        contre la pesanteur moyenne, posture par posture (moindres carrés ;
        1, ils la suivent ; 0, ils n'en ont rien). Rend (fiables, {pentes}) :
        fiables si toutes les pentes sont entre `pente_min` et `pente_max`,
        None si aucune articulation ne varie assez pour en juger.

        Constaté sur GPU (essai T1 du 5 novembre 2026) : au tangage d'épaule,
        pesanteur de -2.6 à -4.8 N·m selon la posture, couples lus à
        quelques dixièmes de zéro ; les « biais » valaient l'opposé de la
        pesanteur. Le banc lisait les efforts imposés aux articulations
        (nuls, elles sont commandées en position), pas le couple des moteurs :
        la force à la main n'était que du bruit."""
        par: Dict[str, Dict[str, list]] = {}
        for posture, rel in self.releves:
            for j, (t, g) in rel.items():
                if math.isfinite(t) and math.isfinite(g):
                    par.setdefault(j, {}).setdefault(posture, []).append((t, g))
        pentes = {}
        for j, pp in par.items():
            moy = np.array([np.mean(v, axis=0) for v in pp.values()])     # (posture, [τ, g])
            if len(moy) < 2 or float(np.ptp(moy[:, 1])) < etendue_min_nm:
                continue
            g, t = moy[:, 1] - moy[:, 1].mean(), moy[:, 0] - moy[:, 0].mean()
            pentes[j] = float(g @ t / (g @ g))
        if not pentes:
            return None, {}
        return all(pente_min <= v <= pente_max for v in pentes.values()), pentes

    def resoudre(self) -> Dict[str, Tuple[float, float, int]]:
        """{articulation: (biais N·m, résidu N·m, relevés)} ; le résidu est
        l'écart quadratique moyen, posture par posture (moyennes des relevés
        de chaque posture), de ce que le biais n'explique pas."""
        ecarts: Dict[str, Dict[str, list]] = {}
        for posture, rel in self.releves:
            for j, (t, g) in rel.items():
                if math.isfinite(t) and math.isfinite(g):
                    ecarts.setdefault(j, {}).setdefault(posture, []).append(t - g)
        sortie = {}
        for j, par_posture in ecarts.items():
            tous = [e for es in par_posture.values() for e in es]
            b = float(np.mean(tous))
            moyennes = [float(np.mean(es)) - b for es in par_posture.values()]
            sortie[j] = (b, float(np.sqrt(np.mean(np.square(moyennes)))), len(tous))
        return sortie

    @staticmethod
    def bilan(biais: Dict[str, Tuple[float, float, int]], postures: int) -> str:
        """Texte lisible : une ligne d'en-tête, puis une par articulation."""
        if not biais:
            return "calibration des couples : aucun relevé — force à la main sans correction"
        pire = max(biais.items(), key=lambda kv: kv[1][1])
        lignes = [f"calibration des couples ({postures} postures) : biais par articulation "
                  f"(couple estimé - pesanteur du modèle), résidu max {pire[1][1]:.2f} N·m "
                  f"({pire[0]})"]
        for j, (b, res, n) in sorted(biais.items()):
            lignes.append(f"    {j:28s} biais {b:+.2f} N·m, résidu {res:.2f} N·m ({n} relevés)")
        return "\n".join(lignes)


class CalibrationMains:
    """Compare, sur quelques postures, les mains calculées par la
    cinématique (angles mesurés) aux mains mesurées par le robot. Le banc
    T1 la fait au démarrage ; au-delà de `ecart_max_m`, la prise mesurée
    est refusée (ManipulationService.calibration_refus)."""

    def __init__(self, modele: ModeleBras, ecart_max_m: float = 0.015):
        self.modele, self.ecart_max_m = modele, ecart_max_m
        self.releves: list = []          # (posture, écart gauche, écart droit)

    def ajouter(self, posture: str, angles: Dict[str, float],
                mains_reelles: Tuple[np.ndarray, np.ndarray]) -> None:
        eg, ed = ecarts_modele(self.modele, angles, mains_reelles)
        self.releves.append((posture, eg, ed))

    @property
    def ecart_max(self) -> float:
        """Plus grand écart relevé (m) ; +inf sans relevé, ou s'il n'est pas fini."""
        if not self.releves:
            return math.inf
        v = max(float(np.linalg.norm(e)) for _, eg, ed in self.releves for e in (eg, ed))
        return v if math.isfinite(v) else math.inf

    def bilan(self) -> Tuple[bool, str]:
        """(acceptée, texte lisible, une ligne par posture et par main)."""
        lignes = []
        for nom, eg, ed in self.releves:
            for cote, e in (("gauche", eg), ("droite", ed)):
                lignes.append(f"    {nom:10s} main {cote:6s} : réel - modèle = "
                              f"({100 * e[0]:+.1f}, {100 * e[1]:+.1f}, {100 * e[2]:+.1f}) cm, "
                              f"{100 * float(np.linalg.norm(e)):.1f} cm")
        ok = self.ecart_max <= self.ecart_max_m
        tete = (f"calibration des mains : écart max {100 * self.ecart_max:.1f} cm "
                f"(limite {100 * self.ecart_max_m:.1f}) — "
                + ("prise mesurée autorisée" if ok else
                   "MODÈLE DES BRAS FAUX, prise mesurée REFUSÉE"))
        return ok, "\n".join([tete] + lignes)
