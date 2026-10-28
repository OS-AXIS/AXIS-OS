# Première mise en route d'un robot sous AXIS-OS

*Conception cible : ce parcours n'est pas encore implémenté. La section
« État actuel » dit ce qui existe aujourd'hui.*

Ce document décrit ce qui se passe entre le premier allumage d'un robot
équipé d'AXIS-OS et le moment où il exécute sa première mission. Il sert de
plan de travail aux chantiers « interface du robot », « carte » et
« jumelage ». Il ne suppose rien du fabricant : tout ce qui suit est fait par
AXIS-OS lui-même, que le robot ait un écran ou non.

## Principes

- **Sûr avant d'être pratique.** Tant que la configuration n'est pas finie,
  le robot ne bouge pas.
- **Aucun mot de passe commun.** Chaque robot génère ses propres secrets au
  premier démarrage : nom de réseau temporaire, mot de passe, code de
  jumelage. Ils sont aléatoires, à usage unique et expirent.
- **Preuve de présence physique.** La première configuration et toute remise
  à zéro exigent une action faite à côté du robot. Être sur le même réseau
  ne suffit pas.
- **Rien d'essentiel ne dépend d'internet.** Le robot se configure, se
  surveille et s'arrête sans notre serveur. Le lien avec le site AXIS-OS est
  facultatif.
- **Rien ne dépend du fabricant.** Pas d'étiquette, d'écran ni de logiciel
  imposé au fabricant.

## 1. Premier allumage : bilan, sans bouger

1. AXIS-OS démarre en **mode sûr** : moteurs bloqués ou en mode souple selon
   le robot, marche impossible, aucune app lancée.
2. **Bilan de santé** : capacités présentes (caméras, micro, haut-parleur,
   lidar, écran, bras, mains), état des moteurs et des articulations,
   batterie, calibrage, présence et fonctionnement de l'arrêt d'urgence.
3. **Génération des secrets** propres à ce robot (voir Principes).
4. Le robot annonce qu'il est prêt, par le moyen dont il dispose : écran,
   voix, lumière.

## 2. Se connecter au robot

AXIS-OS détecte s'il y a un écran, comme toute autre capacité matérielle
(capacité `DISPLAY` ; sous Linux, l'état des sorties vidéo dans
`/sys/class/drm/`).

### Cas A — le robot a un écran

L'écran affiche le nom du réseau Wi-Fi temporaire, un QR code qui ouvre
l'interface du robot, et le code de configuration.

### Cas B — le robot n'a pas d'écran (par exemple le Unitree G1)

- **Voix** : le robot prononce son nom et son code de configuration
  (« Je suis AXIS G1 4F2A. Mon code est 4, 8, 2, 7. ») et le répète sur
  demande.
- **Réseau** : on rejoint son Wi-Fi temporaire ou on le branche par câble.
  Le robot s'annonce sur le réseau (mDNS, par exemple `axis-g1-4f2a.local`),
  et la page de configuration le trouve sans adresse à saisir.
- **Remise par l'installateur** : quand AXIS-OS est installé sur le robot,
  l'outil d'installation peut produire le code de configuration et le
  remettre à la personne (imprimé, par courriel ou dans son compte).
- **Robot sans haut-parleur ni installateur** : console de secours par câble
  (USB ou série), réservée à l'intégrateur.

### Preuve de présence physique (les deux cas)

L'assistant demande une action impossible à distance, par exemple :

- lever doucement un bras du robot, moteurs en mode souple : le robot
  détecte le mouvement ;
- appuyer deux fois sur l'arrêt d'urgence.

## 3. Assistant de configuration (interface du robot, sans internet)

1. Langue, date et heure.
2. Réseau : Wi-Fi de l'entreprise ou câble. Le Wi-Fi temporaire s'éteint
   ensuite.
3. Compte administrateur local (mot de passe ou code) : lui seul peut
   installer des apps et changer les réglages.
4. **Sécurité, étape obligatoire et non contournable** :
   - appuyer sur l'arrêt d'urgence physique et vérifier que le robot l'a
     reçu ;
   - régler la vitesse maximale et les distances du gardien de proximité ;
   - choisir le profil d'exécution : PRODUCTION par défaut sur un robot réel.
5. Premier mouvement guidé, lent, avec une personne à côté et l'arrêt
   d'urgence à portée de main : se lever ou bouger les bras.

## 4. Jumelage avec le site AXIS-OS (facultatif)

1. L'interface du robot affiche (ou dit) un **code de jumelage** à usage
   unique.
2. Sur le site AXIS-OS, la personne choisit « Ajouter un robot » et saisit le
   code.
3. Le robot est lié au compte : suivi à distance, installation d'apps depuis
   le site, mises à jour.

Le site n'envoie jamais de commande aux moteurs. Il demande (« installe
cette app », « lance cette mission ») ; le robot vérifie et décide. Le
jumelage peut être sauté ou retiré à tout moment.

## 5. Carte du lieu

1. **Cartographie** : on promène le robot dans le bâtiment (à la manette ou
   avec « suis-moi ») pendant qu'il construit la carte (SLAM). On peut aussi
   importer une carte existante : celle d'autres robots du client, un plan au
   format LIF (VDMA), ou une maquette du bâtiment.
2. **Lieux nommés** : « quai 3 », « stock », « poste de charge »…
3. **Zones** : zones interdites (le robot n'y entre jamais) et zones lentes,
   appliquées par l'OS, jamais par les apps.

### Où la carte est enregistrée

- **Sur chaque robot**, toujours : il doit naviguer sans réseau.
- **Partage entre robots d'un même site** : par un serveur local du site
  (sur le réseau de l'entreprise) ou, si le client l'accepte, par le cloud
  AXIS-OS. Un seul robot cartographie ; les autres reçoivent la carte.
- La carte est **versionnée** (on peut revenir à la précédente) et
  **validée par l'administrateur du site** avant d'être distribuée.
- Le plan d'un bâtiment est une information sensible pour le client : par
  défaut, elle reste sur le site ; l'envoi vers le cloud est un choix
  explicite, chiffré, et réversible.

## 6. Prêt

- L'**assistant système préinstallé** répond aux commandes de base :
  « suis-moi », « arrête-toi », « va au quai 3 ».
- Installation d'apps : chaque app demande ses permissions, montre ses
  quotas et sa clé de signature, et déclare les **lieux dont elle a besoin**.
  S'il en manque sur la carte, l'OS demande de les placer avant d'activer
  l'app.
- Vérification des **mises à jour de l'OS**, toujours signées.

## État actuel

Rien de ce parcours n'est encore implémenté. Aujourd'hui, AXIS-OS est
développé et validé **en simulation** (Isaac Sim, robot simulé léger des
tests). Briques déjà présentes :

- détection des capacités matérielles (sans la capacité `DISPLAY`) ;
- profils simulation, développement et production ;
- arrêt d'urgence logiciel, garde-fou articulaire, gardien de proximité ;
- signatures, permissions et bac à sable des apps ;
- SDK 1.2 : `go_to` (aussi vers un lieu nommé), `follow`, `speak`
  (factice), `where_am_i`, `notify`, `battery`, `thermal_status`, etc. ;
- carte simple (`Carte`, obstacles écrits dans le code), sans SLAM ;
- carte du sens (`SiteMap`) : lieux nommés, station de charge, zones
  interdites et lentes appliquées par l'OS à toute marche, versionnée,
  écrite par l'administrateur par l'API de l'OS (pas encore d'éditeur) ;
- lieux requis par une app : l'installation est refusée tant qu'ils
  manquent sur la carte (l'OS ne propose pas encore de les placer) ;
- tâches planifiées (`MissionScheduler`), retour à la station de charge
  (`EnergyService`), santé matérielle et protection thermique
  (`HardwareHealthService`, garde-fou articulaire), notifications vers
  l'opérateur (canal factice), rapports de plantage.

Manquants pour qu'un robot réel démarre seul :

- **le point d'entrée du service** : `deploy/systemd/axis-os.service` lance
  `python -m axis_os.main`, qui n'existe pas encore ; c'est lui qui devra
  démarrer l'ordonnanceur des tâches planifiées et le service d'énergie
  (aujourd'hui démarrés par les tests) ;
- la lecture de la batterie du G1 réel (`get_battery` rend « inconnue » :
  le service d'énergie ne fait alors rien) et l'accostage réel à la station ;
- le vrai canal des notifications (écran du robot, téléphone) ;
- l'interface du robot et l'assistant de configuration ;
- la génération des secrets, le Wi-Fi temporaire, l'annonce mDNS ;
- la voix réelle (`speak` est factice) ;
- le SLAM et l'éditeur de carte (lieux et zones se publient aujourd'hui par
  l'API de l'OS) ;
- le serveur du site et le jumelage ;
- la validation sur un vrai robot (pilote G1 en DDS écrit mais jamais testé).

Ordre prévu : interface du robot (étapes 1 à 3), carte (étape 5), puis site
et jumelage (étape 4).
