# Follow me

« Suis-moi » : le robot suit la personne qui est devant lui
(`person_in_front` puis `follow`) et le dit (`speak`). « Arrête-toi » ou
« stop » : il s'arrête (`stop_follow`). Il le dit aussi quand il perd la
personne (`LOST`). Une mission correspond à une séance de suivi.

```bash
python examples/follow_me/test_app.py
```

C'est l'OS qui suit : il garde 1,5 m, au-delà de la zone où le gardien de
proximité arrête le robot ; il marche à 0,25 m/s au plus et ne recule jamais.
Il s'immobilise quand quelqu'un passe tout près, cherche la personne perdue
pendant quelques secondes au plus, et ne change jamais de personne : dans le
doute, il rend `LOST`. L'app donne de ses nouvelles à chaque tour de boucle
(`follow_status`, l'homme mort) : si elle se tait plus de `deadman_s`, l'OS
arrête le suivi.

Le test joue un faux monde (`axis_os.testing`) : une personne qui marche,
s'arrête puis repart de côté, une deuxième qui passe entre elle et le robot,
et un mur derrière lequel on disparaît.

Permissions : `SENSORS_MICROPHONE`, `PEOPLE_DETECT` (NORMAL, montrée à
l'humain : « cette app détecte la présence de personnes »), `PEOPLE_FOLLOW`
(DANGEROUS), `LOCOMOTION_WALK` et `ACTUATORS_WRITE_LEGS` (les droits de
`walk()`), `AUDIO_OUTPUT`, `MISSION_LIFECYCLE`.

Aucune reconnaissance faciale ni identification : l'app ne connaît qu'un
numéro temporaire, une distance et une direction.
