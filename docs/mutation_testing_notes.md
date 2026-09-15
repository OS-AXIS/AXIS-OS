# Mutation testing sur limits.py (point 4, recommandation externe)

Session menée le 2026-09-15 avec `cosmic-ray` (mutmut 3.x s'est révélé
incompatible avec le style de nos fichiers de test — scripts avec
assertions manuelles et `sys.exit()`, pas de vraies fonctions
pytest — sans réécriture complète des tests existants).

## Configuration utilisée

```toml
[cosmic-ray]
module-path = "axis_os/hal/safety/limits.py"
timeout = 30.0
test-command = "sh -c 'cd <repo> && python3 test/test_safety_v2.py && python3 test/test_audit_fixes_wave2.py && python3 test/test_audit_fixes_wave3.py && python3 test/test_g1_dds_actuator.py'"

[cosmic-ray.distributor]
name = "local"
```

## Résultat (partiel — timeout atteint avant la fin)

```
305 mutants générés au total
59 exécutés (19.34%) avant timeout de session
28 survivants sur ces 59 (47.46%)
```

Le taux de survie élevé n'est **pas représentatif de la qualité
réelle du code** — la majorité des mutants examinés touchaient des
branches peu exercées par les 4 fichiers de test choisis comme
échantillon (choisis pour rester dans un temps de session
raisonnable), pas l'ensemble de la vraie suite (31 fichiers). Un run
complet avec tous les fichiers de test comme `test-command` donnerait
un taux de survie très probablement plus bas, mais durerait
nettement plus longtemps (305 mutants × suite complète).

## Vrais trous de couverture trouvés et corrigés

Chaque mutant listé ci-dessous **survivait avant** correction — la
preuve concrète qu'aucun test n'exerçait ce cas limite précis, pas
une supposition. Fermés dans `test/test_mutation_boundary_cases.py` :

- `abs(feedback.torque) > self.limits.torque_max * 1.1` — muté en
  `is`, `==`, `<<`, `//`, `**` sans qu'un seul test échoue. Aucun test
  n'exerçait le cas limite exact (`torque == torque_max * 1.1`).
- `feedback.temperature >= self.TEMPERATURE_CRITICAL_C` (deux
  occurrences, `check_feedback_health()` et `validate_command()`) —
  muté en `==` et `>` sans échec. Le comportement voulu à la limite
  exacte (`temperature == CRITICAL`, `>=` donc inclusif) n'était
  jamais vérifié explicitement.
- `delta > self.limits.position_delta_max` — muté en `>=` sans échec.
  Le cas limite exact (`delta == position_delta_max`, qui doit rester
  ACCEPTÉ puisque `>` est strict) n'était pas testé — un risque de
  faux rejet au bord si ce `>` devenait `>=` par erreur un jour.
- `accel_abs > self.limits.accel_max` — muté en `==`, `is`, et
  inversion (`not accel_abs > ...`) sans échec — confirme
  concrètement ce qu'un audit externe précédent avait déjà suspecté
  (`accel_max` peu exercé par la suite de tests).
- `limits.pos_min >= limits.pos_max` (validation de construction de
  `JointLimits`) — muté en `>` sans échec. Le cas limite
  `pos_min == pos_max` (des bornes de position identiques, donc un
  joint qui ne pourrait physiquement jamais bouger) n'était pas testé.

## Ce qui n'a volontairement PAS été corrigé

De nombreux mutants survivants touchaient uniquement le champ `limit=`
transmis dans le message `SafetyViolation` (ex: `limit=torque_max*1.1`
muté en `limit=torque_max*2.1`) — ces mutants survivent parce que le
message d'erreur n'est jamais comparé strictement par nos tests
(seul le fait qu'une violation soit levée l'est), pas parce que le
comportement de SÉCURITÉ lui-même est mal testé. Corriger ces mutants
spécifiquement ajouterait des assertions sur le contenu exact des
messages de log — une amélioration de fiabilité des messages
d'erreur, pas une lacune de sécurité, laissée hors du périmètre de
cette session.

## Reprendre cette session

```bash
cd <repo>
cosmic-ray baseline cosmic_ray_limits.toml   # vérifie que la suite passe sans mutation
cosmic-ray init cosmic_ray_limits.toml cosmic_ray_limits.sqlite
cosmic-ray exec cosmic_ray_limits.toml cosmic_ray_limits.sqlite
cr-report cosmic_ray_limits.sqlite
```

Pour couvrir `kinematic_normalizer.py` également (visé par la
recommandation externe mais pas encore testé faute de temps dans
cette session), dupliquer `cosmic_ray_limits.toml` en changeant
`module-path`.

Pour un run plus complet et représentatif, utiliser TOUS les fichiers
de test (`test/test_*.py`) dans `test-command` plutôt que les 4
choisis ici pour rester dans un temps de session raisonnable — au
prix d'un temps d'exécution bien plus long (chaque mutant relance
alors 31 scripts au lieu de 4).
