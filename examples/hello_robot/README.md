# Hello Robot

Le robot salue de la main (`play_animation("wave")`) puis dit bonjour
(`speak`), dans la langue réglée par l'utilisateur (`get_setting`).

```bash
python examples/hello_robot/test_app.py      # sur le robot simulé léger
```

Permissions : `ANIMATION_PLAY` (DANGEROUS), `ACTUATORS_READ`,
`ACTUATORS_WRITE_ARMS` (les droits de `move_arms`, qu'une animation ne
dépasse jamais), `AUDIO_OUTPUT`, `APP_SETTINGS`, `SYSTEM_INFO`,
`MISSION_LIFECYCLE`.
