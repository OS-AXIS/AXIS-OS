# Night patrol

Une ronde de nuit, lancée par l'OS chaque soir à 22:00 (déclencheur
`daily` du manifeste, permission `SCHEDULED_MISSIONS`) : le robot passe par
les lieux nommés `ronde 1`, `ronde 2` et `ronde 3` de la carte du site
(`go_to("ronde 1")`). À chaque lieu, il regarde autour de lui : une personne
dans une zone interdite déclenche une notification URGENT (`notify`). Sous
30 % de batterie (`battery`), il le dit et rentre à la station de charge. Un
moteur qui chauffe (`thermal_status` à `throttled` ou pire) : il fait une
pause jusqu'à ce qu'il refroidisse.

```bash
python examples/night_patrol/test_app.py
```

L'OS garde la main sur ce qui compte, que l'app le fasse ou non : il ne
laisse jamais le robot entrer dans une zone interdite, ralentit dans les
zones lentes, bride puis arrête un moteur trop chaud, et rentre lui-même à
la station sous son propre seuil (20 %). Il ne lance jamais la ronde pendant
un arrêt d'urgence, après une chute, hors des heures permises par
l'administrateur, ni pendant une autre mission ; une ronde manquée (robot
éteint à 22:00) n'est pas rattrapée plus tard.

Le manifeste déclare `required_places` : l'installation est refusée tant que
l'administrateur n'a pas placé les trois lieux sur la carte.

Le test joue un faux site (`axis_os.testing`) : trois lieux, une station,
une zone interdite avec quelqu'un dedans ; la ronde lancée par
l'ordonnanceur à 22:00 ; une batterie qui baisse en route ; un moteur qui
chauffe avant le départ.

Permissions : `LOCATION` (NORMAL, montrée à l'humain : « cette app connaît
la position du robot sur la carte du site »), `NAVIGATION_GOTO`,
`LOCOMOTION_WALK` et `ACTUATORS_WRITE_LEGS` (DANGEROUS), `PEOPLE_DETECT`,
`NOTIFICATIONS`, `HARDWARE_HEALTH`, `MISSION_LIFECYCLE`,
`SCHEDULED_MISSIONS` (DANGEROUS).

L'app calcule où se trouve une personne vue (position du robot, distance et
direction) seulement pour savoir si elle est dans une zone interdite ; elle
ne la garde pas et ne sait pas qui c'est.
