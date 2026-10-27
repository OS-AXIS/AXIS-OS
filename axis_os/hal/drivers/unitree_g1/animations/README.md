# Animations fournies avec le G1

Jouées par `ctx.play_animation("wave")` (permission `ANIMATION_PLAY`), et
vérifiées avant de jouer comme toute animation (voir
`axis_os/store/sdk/animation.py` et `axis animation validate <fichier>`).

| Nom | Geste |
|---|---|
| `wave` | salue de la main droite |
| `point` | pointe droit devant du bras droit |

Pas de hochement de tête : le G1 23 DOF n'a pas d'articulation de cou.

Les sens de rotation sont ceux mesurés sur le G1 (le tangage d'épaule
négatif porte le bras vers l'avant ; un coude négatif se plie), et non les
étiquettes de `conventions.py`, connues fausses pour ces deux
articulations. Le sens anatomique absolu reste à contrôler sur le robot
réel avant usage (docs/PREREQUIS_ROBOT.md).
