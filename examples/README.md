# Exemples du SDK AXIS-OS

Quatre apps complètes, chacune avec son test sur le robot simulé léger
(`axis_os.testing`), lancés par la suite (`test/test_exemples_sdk.py`) :

| Exemple | Ce qu'il montre |
|---|---|
| [hello_robot](hello_robot/) | `play_animation` + `speak`, réglage de langue |
| [fetch](fetch/) | `look_for` + `go_to` + `pick` + `place` + `rest_pose` |
| [teleop_local](teleop_local/) | `stream_targets` sous homme mort, manette factice |
| [follow_me](follow_me/) | « suis-moi » à la voix : `person_in_front` + `follow` + `speak` |

Référence de chaque API : [docs/API_REFERENCE.md](../docs/API_REFERENCE.md).
