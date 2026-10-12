# Audit avant publication — secrets et données personnelles

**Date : 2026-10-11.** Objet : vérifier, avant de rendre le dépôt public,
qu'aucun secret ni aucune donnée personnelle n'est publié, ni dans les
fichiers actuels ni dans l'historique git.

Ce document ne recopie aucune valeur sensible : il nomme le fichier, le
commit et le type de donnée, jamais la donnée elle-même.

Les empreintes de commit citées sont celles de l'historique **d'avant
réécriture** (2026-10-11). La réécriture prévue (voir « Traitement ») change
toutes les empreintes ; les dates et les chemins, eux, restent valables.

## Résultat en bref

- **Aucun secret trouvé** — ni jeton, ni clé privée, ni mot de passe, ni URL
  contenant des identifiants — dans les fichiers actuels comme dans les
  249 commits de l'historique (2 branches, aucun tag).
- **Quatre données personnelles trouvées**, toutes hors du code exécuté :
  1. une adresse e-mail personnelle dans les métadonnées (auteur et
     committeur) de 25 commits ;
  2. une deuxième adresse e-mail personnelle dans les fichiers de l'app
     d'exemple `mon_app/` (fichiers actuels et historique) ;
  3. le nom d'hôte interne d'une machine de cloud dans l'identité d'un
     commit ;
  4. des chemins absolus de l'appareil du développeur, compilés dans
     d'anciens fichiers `.pyc` suivis par erreur.

Le traitement de chacune est décrit plus bas. Tant qu'il n'est pas fait, le
dépôt ne doit pas être rendu public.

## Méthode

Le dépôt a d'abord été récupéré en entier (`git fetch --unshallow`) : un
clone partiel n'aurait montré que les 50 derniers commits. Toutes les
analyses portent sur l'ensemble des objets accessibles depuis toutes les
références (`git rev-list --all --objects`), c'est-à-dire chaque version de
chaque fichier ayant existé, ainsi que les messages et les identités des
commits.

### Outils automatiques

| Outil | Version | Contrôle du binaire | Portée |
|-------|---------|---------------------|--------|
| gitleaks | 8.30.1 | SHA-256 de l'archive vérifiée contre le fichier de sommes publié avec la version | tout l'historique (`gitleaks git --log-opts=--all`) et l'arbre de travail (`gitleaks dir`) |
| trufflehog | 3.97.9 | SHA-256 de l'archive vérifiée de la même façon | tout l'historique (`trufflehog git file://…`), option `--no-verification` |

`--no-verification` empêche trufflehog d'essayer un éventuel secret trouvé
auprès du service concerné : un audit ne doit pas utiliser ce qu'il trouve.

Résultat : **0 fuite** pour gitleaks, **0 secret** (vérifié ou non) pour
trufflehog.

### Recherches ciblées

Les outils automatiques connaissent les formats des secrets, pas les
données personnelles propres à ce projet. Un script a donc cherché, dans
chaque version de chaque fichier et dans chaque message de commit :

| Recherche | Résultat | Verdict |
|-----------|----------|---------|
| Jetons GitHub (`ghp_`, `gho_`, `github_pat_`), Hugging Face (`hf_`), Anthropic, AWS, Slack | 0 | — |
| Clés privées (`BEGIN … PRIVATE KEY`, clés SSH) | 0 | — |
| Affectations « token / password / secret = … » | 1 | valeur factice d'un test (`test/test_vla_gr00t_connector.py`), sans valeur réelle |
| URL contenant un identifiant (`schéma://utilisateur:mot@hôte`) | 0 | — |
| Adresses IP publiques | 2 valeurs | une chaîne de version (`docs/isaac_lab_unitree_sim_status.md`) et une adresse d'exemple dans un commentaire (`deploy/machine_gpu/lancer_t1.sh`) |
| Hôtes et ports Vast.ai, commandes `ssh` | le mot « Vast.ai » seulement | aucune adresse ni aucun port d'instance ; la commande `ssh` de `lancer_t1.sh` est un exemple à remplir |
| Adresses e-mail dans les fichiers | 14 valeurs | 13 adresses d'exemple ou de test (domaines `example`, `exemple`, `axis-os.local`, `t@t.com`…) et **une adresse personnelle** (donnée n° 2) |
| Adresses e-mail dans les identités de commit | 7 identités | voir données n° 1 et n° 3 |
| Numéros de téléphone | 2 263 correspondances, toutes dans les anciens `logs/*.jsonl` | faux positifs : suites de chiffres à l'intérieur d'empreintes et d'horodatages ; 0 ailleurs |
| Adresses postales, code postal suivi d'une ville | 1 correspondance | faux positif : référence arXiv dans `docs/fleet_manager_design.md` |
| Chemins personnels (`/home/<nom>`, `/Users/…`, dossiers d'applications mobiles) | 38 fichiers, tous des `.pyc` | donnée n° 4 ; aucun chemin personnel dans un fichier source |
| Nom de famille associé à l'adresse personnelle | uniquement dans l'adresse elle-même (donnée n° 1) | — |

Le prénom de l'auteur (« Kenzo ») apparaît dans quelques exemples et
commentaires (`docs/CREER_UNE_APP.md`, `mon_app/`, un commentaire du service
de manipulation). C'est le nom sous lequel le projet est signé : il reste.

### Dossiers et scripts regardés en particulier

- **`logs/`** — `estop.jsonl`, `safety_violation.jsonl` et
  `watchdog_alert.jsonl` ont été suivis du 2026-09-05 (`8a8e4d4`) au
  2026-10-10 (`2749d4a`), en 120 versions et 128 263 lignes au total. Chaque
  ligne a été lue comme du JSON : ce ne sont que des sorties de la suite de
  tests (catégorie, identifiant de robot de test comme `g1_wdg`, angles
  articulaires, règle de sécurité déclenchée). Aucun chemin, aucune IP,
  aucun nom d'hôte, aucun nom. **Rien de sensible** ; ils ne sont plus
  suivis depuis le 2026-10-10 (`.gitignore`).
- **`diag_*.py`, `video_*.py`** — aucun chemin absolu, aucune IP, aucun hôte,
  aucun identifiant. `sim/video/video_v2_voix.py` télécharge à l'exécution le modèle
  de synthèse vocale `facebook/mms-tts-fra`, dont la licence interdit
  l'usage commercial : c'est signalé dans `THIRD_PARTY.md`, le modèle n'est pas
  dans le dépôt.
- **Fichiers supprimés depuis** (`README-1.md`, `requirments.txt`, anciens
  `.pyc`) : seuls les `.pyc` contiennent une donnée personnelle (n° 4).
- **`Drive/`** (ignoré par `.gitignore`) : n'a jamais été suivi.

## Détail des données personnelles et traitement

| # | Donnée | Où | Traitement |
|---|--------|----|------------|
| 1 | Adresse e-mail personnelle (Gmail) de l'auteur | identité auteur et committeur de 25 commits, nom affiché « Kenzo », du 2026-09-26 au 2026-09-29 | réécriture de l'historique avec un `mailmap` : remplacée par l'adresse publique du projet, `axis.project.dev@gmail.com` |
| 2 | Deuxième adresse e-mail personnelle (Gmail) | `mon_app/app.py` (champ `author_email`), `mon_app/manifest.json`, et l'ancien `mon_app/__pycache__/app.cpython-314.pyc` | remplacée par l'adresse publique dans les fichiers actuels (commit normal), puis dans tout l'historique à la réécriture ; le commit de remplacement disparaît alors, devenu sans objet |
| 3 | Nom d'hôte interne d'une machine de cloud (format `ip-…compute.internal`), qui révèle une adresse IP privée et une région | identité auteur et committeur d'un commit du 2026-09-04 (nom affiché « Ubuntu ») | réécriture : identité remplacée par « Kenzo » et l'adresse publique |
| 4 | Chemins absolus de l'appareil personnel du développeur | 38 fichiers `__pycache__/*.pyc` ajoutés le 2026-09-03 et le 2026-09-04, retirés du suivi depuis | réécriture : tous les `.pyc` et dossiers `__pycache__` supprimés de l'historique |

Les commits signés « Claude (assistant) » ou « Claude » utilisent des
adresses génériques d'Anthropic, ceux signés « OS-AXIS » l'adresse publique
du projet : rien de personnel.

## Ce que la réécriture ne suffit pas à effacer

Ces points ne relèvent pas du dépôt lui-même. Ils sont à régler avant de
rendre le dépôt public, et chacun demande une décision du propriétaire.

1. **La sauvegarde de l'ancien historique.** Le tag `avant-publication`
   posé avant la réécriture pointe vers l'ancien historique, données
   personnelles comprises. Il doit rester sur une copie privée et ne jamais
   être poussé vers le dépôt public.
2. **Les commits que GitHub garde en cache.** Après un push forcé, GitHub
   peut encore servir un ancien commit à qui connaît son empreinte, tant
   qu'il n'a pas fait le ménage de ses objets. Le dépôt n'a aujourd'hui ni
   pull request ni fork (vérifié le 2026-10-11), donc aucune référence
   cachée (`refs/pull/*`) ne retient l'ancien historique. Deux façons sûres
   de conclure : publier la nouvelle histoire dans un dépôt **neuf**, ou,
   après le push forcé, demander au support GitHub de purger les vues en
   cache avant de passer le dépôt en public.
3. **Toutes les branches distantes.** La branche
   `claude/intelligent-mendel-5jhqxo` porte elle aussi l'ancien historique :
   elle doit être supprimée ou remplacée en même temps que `main`.
4. **Les clones existants** (machine GPU, poste de travail) gardent l'ancien
   historique jusqu'à ce qu'ils soient supprimés et reclonés.
5. **Les releases brouillons.** Le dépôt a une release en brouillon
   (`vla-episodes`, épisodes de démonstration enregistrés en simulation),
   créée par `deploy/machine_gpu/sauvegarder_vla.sh`.
   `sauvegarder_modele_vla.sh` peut en créer une seconde contenant le modèle
   VLA affiné, donc dérivé des poids GR00T de NVIDIA, que leur licence
   encadre. Un brouillon n'est visible que des personnes qui ont un accès
   en écriture, y compris quand le dépôt est public : ces deux releases
   doivent rester des brouillons, ou être déplacées vers un stockage privé.
   D'après les scripts qui les produisent, les épisodes ne contiennent que
   des données de simulation ; l'archive elle-même n'a pas été ouverte pour
   cet audit.

## Refaire cet audit

Avant chaque publication importante, et dans tous les cas après la
réécriture :

```sh
gitleaks git --log-opts="--all" .      # tout l'historique
gitleaks dir .                         # arbre de travail
trufflehog git file://$PWD --no-verification
```

Puis vérifier à la main que l'adresse personnelle n'apparaît plus nulle
part : `git log --all --format='%an %ae %cn %ce %B'` et
`git grep -i <adresse> $(git rev-list --all)` ne doivent rien rendre.
