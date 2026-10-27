# Fetch

Va chercher un objet et le rapporte : `look_for(..., "world")`, `go_to`
devant l'objet, `pick`, `go_to` au point de dépose, `place`, `rest_pose`.
Chaque échec est rendu avec son code de refus (voir
[API_REFERENCE.md](../../docs/API_REFERENCE.md)).

```bash
python examples/fetch/test_app.py      # sur le robot simulé léger
```

Permissions : `NAVIGATION_GOTO` et `MANIPULATION` (DANGEROUS), plus les
droits des commandes qu'elles remplacent (`LOCOMOTION_WALK`,
`ACTUATORS_WRITE_LEGS` pour `go_to` ; `ACTUATORS_READ`,
`ACTUATORS_WRITE_ARMS` pour `pick`, `place` et `rest_pose`),
`SENSORS_CAMERA`, `SYSTEM_INFO`, `MISSION_LIFECYCLE`.
