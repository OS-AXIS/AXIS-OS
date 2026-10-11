# Démonstration multi-apps — 2026-09-24

Mesures réelles contre Isaac Sim 6.1.0, G1 23 DOF debout sous politique
de marche, vrai Play Store, vrai sandbox, vrai garde-fou. Reproductible
via `sim/validation/test_isaac_d1_multiapp.py`.

---

## 1. Résultat

**Trois apps tierces cohabitent sur un même robot, et chacune est
limitée à ce qu'elle a déclaré.** Cinq scènes, quatre refus, aucun effet
de bord, robot debout du début à la fin.

| Scène | Ce qui est tenté | Réponse du système |
|---|---|---|
| 1. Catalogue | Installer trois apps | Installées, permissions affichées, niveau DANGEREUX signalé |
| 2. Matériel | App exigeant deux mains | **Refusée à l'installation** — le G1 23 DOF n'en a pas |
| 3. Permission | App « caméra » qui tente de marcher | **PermissionError**, robot immobile (0.043 m) |
| 4. Concurrence | Deux missions simultanées | **Refus en 0.00 s**, motif : verrou de mission |
| 5. Commande | Genou à 4.88 rad (butée 2.88) | **ERROR_LIMIT**, violation journalisée |

---

## 2. Ce que cette démonstration prouve

Aucune scène n'est spectaculaire isolément. Ensemble, elles montrent ce
qu'un robot livré avec son logiciel ne peut pas offrir : **plusieurs
applications tierces sur une même machine, sans qu'aucune puisse
dépasser ce qu'on lui a accordé.**

Chaque refus a une cause **différente** et un niveau **différent** :

- à l'**installation** (capacité matérielle absente) ;
- à l'**appel** (permission non déclarée) ;
- à l'**exécution** (robot déjà occupé) ;
- à la **commande** (consigne hors butée).

C'est une défense en profondeur : une app qui franchirait un niveau par
erreur de conception se heurterait au suivant.

---

## 3. La scène 4 est passée au vert trois fois à tort

Le point le plus important de ce document, et il concerne la méthode de
test plutôt que le code.

| Tentative | Verdict affiché | Réalité |
|---|---|---|
| 1 | ✓ refus | les apps n'avaient **pas démarré** (classe non transmissible au sandbox) |
| 2 | ✓ refus | idem, détecté seulement en relisant les journaux |
| 3 | ✓ refus | l'app occupante était **morte en 0.30 s** ; la concurrente avait **tourné 20.34 s** avant d'échouer faute de colis |
| 4 | ✓ refus | **réel** : occupante vivante, refus en 0.00 s, message du verrou |

Trois fois, le test vérifiait **qu'il y avait** un refus, jamais
**pourquoi**. Un échec quelconque était pris pour la preuve recherchée.

**Règle retenue :** un test de refus doit établir le *motif* du refus,
pas seulement son existence. Ici : l'app occupante tourne encore, le
refus est immédiat, et le message est celui du verrou.

C'est le même principe que le test différentiel de la perception et le
contrôle de déplacement réel de l'app d'inspection : ne pas croire ce que
le système déclare, vérifier ce qu'il a fait.

---

## 4. Trois corrections révélées en chemin

**Le type d'une erreur doit traverser le pont.** Une permission manquante
levait `PermissionError` en direct, mais devenait un `RuntimeError`
générique à travers le sandbox — que le client avalait en rendant
`False`. L'app ne pouvait pas distinguer « je n'ai pas le droit » de « ce
robot n'a pas de jambes ». Corrigé par un préfixe `PERMISSION_DENIED:`,
sur le modèle de `ESTOP_ACTIVE:` déjà existant.

**Une app doit être un module importable.** Le sandbox exécute l'app
dans un sous-processus, qui doit pouvoir importer sa classe. Une classe
définie dans une fonction, ou dans le script principal, échoue en 0.3 s
avec `SANDBOX_ERROR` — sans rien indiquer de la cause.

**Le quota mémoire par défaut (256 Mo) est trop bas** dans
l'environnement Python d'Isaac Sim. La même app échoue en 0.30 s avec
256 Mo et réussit en 6.31 s avec 8192 Mo. Ce défaut mérite d'être revu,
ou au moins documenté pour les développeurs tiers.

---

## 5. Limites

- **Un seul robot.** L'arbitrage entre plusieurs robots d'une flotte
  n'est pas couvert.
- **Pas de priorité** : le verrou refuse la seconde mission, il ne met
  pas en file et ne préempte pas. Une mission urgente ne peut pas
  interrompre une mission de routine.
- **Pas d'interface utilisateur** : dans ce test, l'accord des
  permissions DANGEROUS est donné explicitement par
  `accepter_tout_SIMULATION` (simulation seulement, refusé sur un G1 en
  DDS). Sur un vrai robot, sans confirmation l'installation est refusée :
  `confirmer_au_terminal`, ou un écran de confirmation.
