# Contributing to AXIS-OS

*Version française plus bas.*

Thank you for your interest. Issues and pull requests are welcome in
English or French. The code, its comments and the documents in `docs/` are
written in French.

## Before you open a pull request

1. Run the test suite; it needs no GPU (see the README):
   `pip install --require-hashes -r requirements-ci.txt`, then `pytest test/`.
2. Add a test for what you change. For anything related to safety, the most
   useful test shows that the forbidden thing **fails**: "the app tries, and
   is refused".
3. Keep one change per pull request, and explain *why* in its description.

## Safety-critical code

`JointSafetyGuard`, the proximity guard (`GardienProximite`), the watchdog,
`VLAService` and the Play Store permissions are what stands between a bug
and a moving robot. A change to their behaviour needs a dedicated test and
an explanation of the risk. A change that only rewrites comments or
docstrings must leave the code identical: check it with
`python3 outils/verifier_code_inchange.py origin/main HEAD`, which compares
the whole branch with `main`. Without arguments, the tool only checks the
last commit.

To report a vulnerability, do not open an issue: see [SECURITY.md](SECURITY.md).

## Commit messages

- **Subject**: short and natural, in French, in the present tense, saying
  what the change brings, not how. For example: « Arrête le robot quand le
  lidar ne répond plus », rather than « fix: lidar timeout ».
- **Body**, after a blank line: why the change is needed (the problem, how
  it was found), then what it does, and how it was checked.
- Credit co-authors with a `Co-Authored-By:` line at the end.

## License

By contributing, you agree that your contribution is released under the
Apache License 2.0, like the rest of the project (see [LICENSE](LICENSE)).
Everyone taking part follows the [code of conduct](CODE_OF_CONDUCT.md).

---

# Contribuer à AXIS-OS

Merci de votre intérêt. Issues et pull requests sont bienvenues en français
ou en anglais. Le code, ses commentaires et les documents de `docs/` sont
écrits en français.

## Avant d'ouvrir une pull request

1. Lancez la suite de tests, sans GPU (voir le README) :
   `pip install --require-hashes -r requirements-ci.txt`, puis `pytest test/`.
2. Ajoutez un test pour ce que vous changez. Pour tout ce qui touche à la
   sécurité, le test le plus utile montre que la chose interdite
   **échoue** : « l'app essaie, et elle est refusée ».
3. Une pull request par changement, avec le *pourquoi* dans sa
   description.

## Code critique pour la sécurité

`JointSafetyGuard`, le gardien de proximité (`GardienProximite`), le
watchdog, `VLAService` et les permissions du Play Store sont ce qui se tient
entre un bug et un robot qui bouge. Un changement de leur comportement
demande un test dédié et une explication du risque. Un changement qui ne
réécrit que des commentaires ou des docstrings doit laisser le code
identique : vérifiez-le avec
`python3 outils/verifier_code_inchange.py origin/main HEAD`, qui compare
toute la branche à `main`. Sans argument, l'outil ne vérifie que le
dernier commit.

Pour signaler une faille, n'ouvrez pas d'issue : voir [SECURITY.md](SECURITY.md).

## Messages de commit

- **Sujet** : court et naturel, en français, au présent, qui dit ce que le
  changement apporte, pas comment. Par exemple : « Arrête le robot quand le
  lidar ne répond plus », plutôt que « fix: lidar timeout ».
- **Corps**, après une ligne vide : pourquoi le changement est nécessaire
  (le problème, comment on l'a trouvé), puis ce qu'il fait, et comment il a
  été vérifié.
- Les co-auteurs sont cités par une ligne `Co-Authored-By:` à la fin.

## Licence

En contribuant, vous acceptez que votre contribution soit publiée sous la
licence Apache 2.0, comme le reste du projet (voir [LICENSE](LICENSE)).
Chaque participant suit le [code de conduite](CODE_OF_CONDUCT.md).
