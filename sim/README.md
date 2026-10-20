# Scripts de simulation (Isaac Sim)

Ces scripts font tourner AXIS-OS contre un Unitree G1 simulé dans NVIDIA
Isaac Sim 6.1. Ils ne s'exécutent pas sur une machine ordinaire : il faut
le conteneur Isaac Sim et un GPU NVIDIA (en pratique, une instance louée).
C'est pourquoi ils ne font pas partie de la suite `pytest test/`, qui, elle,
tourne partout sans GPU.

Les résultats obtenus avec ces scripts sont décrits, échecs compris, dans
les documents de `docs/` (voir le tableau).

## Ce que contient chaque dossier

| Dossier | Rôle |
|---------|------|
| `validation/` | une **suite de validation** par capacité : le script monte la scène, lance l'OS et ses apps, puis affiche ce qu'il mesure ; la plupart concluent par une liste de vérifications réussies ou échouées |
| `diagnostic/` | des outils pour comprendre une scène qui se comporte mal (que voit la caméra, que contient vraiment la scène USD) ; ils ne valident rien |
| `video/` | les scripts qui tournent les vidéos de démonstration |

Trois autres suites, plus anciennes (`test_s5_*`), sont dans
`deploy/isaac_sim_validation/`, avec leur propre mode d'emploi.

## Suites de validation

Chaque suite, sauf `test_isaac_follower_latency.py`, porte un code, une
lettre et un numéro, repris dans le nom de son fichier ; les scripts se
citent entre eux par ce seul code (« seuil de W4 », « constaté en A3 »).
La lettre donne le thème :

| Code | Thème |
|------|-------|
| L1 à L3 | locomotion : marche, virage, service de marche |
| P1 à P3 | perception |
| A1 à A3 | premières apps de bout en bout : inspection, commande vocale, inventaire |
| D1 | démonstration multi-apps |
| W1 à W4 | GR00T Whole-Body Control (WBC) : marche sous charge, prise et port d'un carton |
| T1 | démonstration de tri de colis |
| S2, S5 | suites de la campagne de validation initiale ([`docs/campagne_validation_isaac.md`](../docs/campagne_validation_isaac.md)) : S2 normalisation, S5 pile complète et endurance ; les sous-numéros des scripts (S5.1, S5.2…) ne suivent pas toujours ceux de ce document |

Ce que vérifie chaque script, et où en lire le résultat :

| Script | Ce qui est vérifié | Résultat |
|--------|--------------------|----------|
| `test_isaac_l1_locomotion.py`, `l2_turning`, `l3_service` | marche, virage, arrêt sûr quand une app plante en pleine marche | `docs/validation_locomotion.md` |
| `test_isaac_p1_perception.py`, `p2_detection`, `p3_warehouse` | perception à vocabulaire ouvert (OWL-ViT) et distance des objets | `docs/validation_perception.md` |
| `test_isaac_a1_inspection.py` | première app qui voit et se déplace | — |
| `test_isaac_a2_voice.py` | commande vocale | `docs/validation_voix.md` |
| `test_isaac_a3_inventaire.py` | inventaire et mémoire des apps | `docs/validation_inventaire.md` |
| `test_isaac_d1_multiapp.py` | plusieurs apps sur un même robot | `docs/validation_multiapp.md` |
| `test_isaac_s2_normalization.py` | cohérence des signes, en position et en couple, contre la physique ; malgré son nom, le script travaille en angles bruts, sans la table de conventions | `docs/groot_architecture.md` |
| `test_isaac_w1_gr00t_port.py` à `w4_gr00t_autonome` | marche sous charge (GR00T Whole-Body Control), prise et port d'un carton | `docs/integration_gr00t_wbc.md`, `docs/validation_port_carton.md` |
| `test_isaac_t1_tri_usine.py` | démonstration de tri de colis en entrepôt, avec des humains | `docs/demo_tri_usine.md` |
| `test_isaac_follower_latency.py`, `test_s5_latency_probe.py` | coût d'une commande, sans puis avec le sandbox | `docs/latence_chemin_commande.md` |

## Les lancer

Les scripts attendent le code d'AXIS-OS dans le conteneur, sous
`/isaac-sim/axis_os/` : ils y cherchent leurs modules par des chemins
absolus, pas par leur place dans ce dépôt.

Sur la machine GPU, `deploy/machine_gpu/copier_code.sh` met à jour le code
du conteneur et y copie les suites `validation/test_isaac_*.py`. Ensuite,
par exemple :

```sh
sudo docker exec isim-isaac-sim-1 /isaac-sim/python.sh \
    /isaac-sim/axis_os/test_isaac_l1_locomotion.py
```

La démonstration T1 a son propre lanceur, `deploy/machine_gpu/lancer_t1.sh`.
Les scripts de `diagnostic/` et `video/`, ainsi que
`validation/test_s5_latency_probe.py`, se copient à la main de la même
façon (`docker cp` vers `/isaac-sim/axis_os/`).
