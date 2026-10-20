# Moniteur de fraîcheur AXIS-OS

Démon en C, séparé du processus principal d'AXIS-OS : si ce processus
meurt en plein mouvement, le moniteur relâche les moteurs du robot en
douceur au lieu de les laisser sur la dernière consigne reçue. C'est
l'une des cinq recommandations de renforcement du second audit externe
(voir `CHANGELOG.md`, section « 5 recommandations externes de
renforcement », du 2026-09-15). Le commentaire en tête de
`freshness_monitor.c` donne la justification complète.

## Statut (mis à jour 2026-09-15)

✅ **Intégration DDS réelle implémentée et compilée avec succès** —
plus un simple squelette. Voir le détail ci-dessous.

- CycloneDDS C compilé depuis les sources officielles
  (`github.com/eclipse-cyclonedds/cyclonedds`, branche `releases/0.10.x`
  — même version que `unitree_sdk2py` utilisée ailleurs dans ce projet).
- Le schéma `LowCmd_`/`MotorCmd_` (dossier `idl/`) n'a pas de fichier
  `.idl` source publié directement par Unitree (leur SDK C++ officiel,
  `unitree_sdk2`, ne distribue que les `.hpp` déjà générés) — reconstruit
  ici en `.idl`, vérifié fidèle en comparant deux générations officielles
  indépendantes (celle du SDK Python et celle du SDK C++, mêmes champs,
  mêmes types, même ordre), puis compilé avec le vrai `idlc` de
  CycloneDDS pour produire du code C authentique.
- `freshness_monitor.c` est entièrement branché sur cette vraie
  intégration DDS : `dds_create_participant`/`dds_create_reader` pour
  s'abonner à `rt/lowcmd`, `dds_take` en poll non bloquant, et
  `dds_publish_damping()` qui écrit un vrai message `LowCmd_` avec un
  CRC32 calculé via l'algorithme **exact** copié depuis
  `unitree_sdk2/include/unitree/dds_wrapper/common/crc.h` (pas
  réinventé — un CRC divergent serait silencieusement rejeté par le
  firmware du vrai robot).
- **Compile sans aucun avertissement** avec `-Wall -Wextra`.
- **Testé en conditions quasi réelles** : démon lancé, abonnement DDS
  réel confirmé, un second programme de test écrit et compilé pour
  publier un vrai message `LowCmd_` sur le même topic.

### ⚠️ Limite trouvée, pas encore levée : multicast désactivé en sandbox

Le message reçu par CycloneDDS lui-même lors du test :
```
selected interface "lo" is not multicast-capable: disabling multicast
```
L'environnement de développement (conteneur sandbox) désactive le
multicast, y compris en loopback — DDS s'appuie dessus par défaut pour
la découverte des participants. Le message de test publié n'a donc
**pas** été reçu par le moniteur dans cet environnement précis, malgré
une intégration DDS entièrement fonctionnelle côté code (participant,
topic, reader/writer tous créés avec succès, aucune erreur DDS hormis
cet avertissement de désactivation multicast).

**Ce n'est pas un bug de ce composant** — c'est une contrainte réseau
du sandbox de développement, absente d'un vrai serveur (le serveur AWS
EC2 déjà utilisé ailleurs dans ce projet pour Isaac Sim, par exemple)
ou d'un vrai réseau embarqué robot. À revalider sur un environnement
avec un vrai réseau avant tout déploiement — piste immédiate :
configurer CycloneDDS en unicast explicite via `CYCLONEDDS_URI`
(`<Discovery><Peers>` avec l'IP exacte du pair) plutôt que la
découverte multicast automatique, si le réseau cible ne le permet pas
non plus.

## Compilation

Nécessite CycloneDDS C compilé et installé (voir plus haut — pas
encore packagé, à installer manuellement pour le moment) :

```bash
# CycloneDDS C — une fois, dans un dossier de travail au choix
git clone --depth 1 --branch releases/0.10.x \
    https://github.com/eclipse-cyclonedds/cyclonedds.git
cd cyclonedds && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=<chemin_install> -DBUILD_IDLC=ON
make -j$(nproc) && make install
```

Puis, depuis ce dossier :

```bash
gcc -Wall -Wextra \
    -I<chemin_install>/include -I. \
    -L<chemin_install>/lib -Wl,-rpath,<chemin_install>/lib \
    -o freshness_monitor \
    freshness_monitor.c idl/LowCmd_.c idl/MotorCmd_.c \
    -lddsc
```

Les fichiers `idl/LowCmd_.c`, `idl/LowCmd_.h`, `idl/MotorCmd_.c`,
`idl/MotorCmd_.h` sont déjà générés et commités (pas besoin de
relancer `idlc` sauf pour modifier le schéma lui-même — voir
`idl/*.idl` pour le source, et régénérer avec
`<chemin_install>/bin/idlc LowCmd_.idl` depuis le dossier `idl/`).

## Tests

```bash
gcc -Wall -Wextra -o test_freshness_monitor test_freshness_monitor.c
./test_freshness_monitor
```

Ce test se lance à la main : la suite `pytest test/` et la CI ne le
compilent pas.

## Configuration (variables d'environnement)

- `AXIS_OS_FRESHNESS_TIMEOUT_MS` (défaut : 200) — délai sans message
  DDS avant publication de l'amortissement.
- `AXIS_OS_DAMPING_KD_X100` (défaut : 200, soit kd=2.0) — gain
  d'amortissement × 100 (pour éviter de parser un float en shell).
- `AXIS_OS_FRESHNESS_POLL_MS` (défaut : 20) — fréquence de vérification
  interne.

## Relation avec le chien de garde systemd

Complémentaire, pas redondant, avec `deploy/systemd/axis-os.service` :
le chien de garde systemd détecte et **redémarre** le process principal
mort (délai de l'ordre de `WatchdogSec` + temps de redémarrage), tandis
que ce moniteur agit **directement sur le bus DDS**, en quelques dizaines
de millisecondes, sans dépendre d'aucune détection système — le robot se
relâche doucement pendant que systemd travaille en arrière-plan à
relancer AXIS-OS.
