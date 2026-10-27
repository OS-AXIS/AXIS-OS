# Teleop local

Téléopération **locale** de la marche par `ctx.stream_targets("walk")`,
sous homme mort, pilotée ici par une manette factice. Bouton relâché, ou
consignes qui n'arrivent plus : l'OS ralentit puis arrête le robot, et
n'accepte plus rien avant `resume()`. Les consignes trop vieilles, trop
fréquentes ou au-delà de la vitesse du flux sont jetées et comptées
(`metrics()`).

```bash
python examples/teleop_local/test_app.py
```

Permissions : `TELEOP_STREAM` (DANGEROUS), `LOCOMOTION_WALK`,
`ACTUATORS_WRITE_LEGS` (les droits de `walk()`), `MISSION_LIFECYCLE`.

Hors périmètre : téléopération par internet, WebRTC, vidéo, casque VR.
