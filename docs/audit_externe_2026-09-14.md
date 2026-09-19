# Audit AXIS-OS — 14 septembre 2026

Repo audité : `OS-AXIS/AXIS-OS` @ `7d878da`. ~14 000 lignes Python, 107 fichiers.
Lu : arborescence complète, `axis_os/hal/`, `axis_os/store/`, `axis_os/apps/`,
`test/`, les 5 documents de `docs/`, le `CHANGELOG.md`, et le fichier URDF officiel.
Les failles marquées **[vérifié]** ont été confirmées par exécution réelle du code,
pas seulement par lecture.

Verdict court : les corrections des audits précédents sont bien en place et
correctement implémentées (aucune régression trouvée). Mais **cinq problèmes
critiques** subsistent, dont quatre sur le chemin qui mène aux actionneurs
physiques. Deux d'entre eux feraient qu'un G1 réel soit soit inpilotable, soit
dangereux dès la première mise sous tension. Ils sont tous corrigeables
rapidement.

---

## 1. CRITIQUE — risque de blessure ou de casse matérielle

### C1. Une position aberrante atteint le moteur sans aucune validation **[vérifié]**

**Fichiers :** `axis_os/hal/safety/limits.py:193` et
`axis_os/hal/drivers/unitree_g1/g1_dds_actuator.py:326`

`JointSafetyGuard.validate_command()` ne vérifie la position **que** si
`command.mode == ControlMode.POSITION` :

```python
if command.mode == ControlMode.POSITION and command.position is not None:
    if not (self.limits.pos_min <= command.position <= self.limits.pos_max):
```

Mais `G1DDSActuator.send_command()` transmet `command.position` au moteur
**quel que soit le mode** :

```python
q=command.position if command.position is not None else 0.0,
```

Les autres champs (vitesse, couple, courant) sont tous validés inconditionnellement.
La position est le seul à être conditionné au mode — et c'est précisément celui
qui est transmis inconditionnellement.

Vérification exécutée :

```
right_knee limits: pos_min=-0.087, pos_max=2.88
A)  mode=TORQUE,   position=500 rad  -> (True, None)   ← ACCEPTÉ
A2) mode=VELOCITY, position=500 rad  -> (True, None)   ← ACCEPTÉ
```

**Scénario concret.** Une app fait du contrôle en couple sur les jambes (cas
normal pour l'équilibre) et réutilise un objet `ActuatorCommand` dans lequel
`position` a gardé une valeur d'une phase précédente, ou reçoit une valeur
aberrante d'un modèle VLA. La commande passe le guard. `q=500.0` part vers le
moteur du genou droit avec `kp=100` : le contrôleur PD interne du G1 voit une
erreur de position de ~497 rad et sature instantanément son couple. Genou en
butée à pleine puissance, robot à terre, réducteur potentiellement détruit.

**Correction.** Valider la position dès qu'elle est non-`None`, indépendamment du
mode — et symétriquement, ne transmettre `q` que dans les modes où elle a un sens
(`POSITION`, `IMPEDANCE`) :

```python
# limits.py
if command.position is not None:
    if not (self.limits.pos_min <= command.position <= self.limits.pos_max):
        ...

# g1_dds_actuator.py
send_position = command.mode in (ControlMode.POSITION, ControlMode.IMPEDANCE)
q = command.position if (send_position and command.position is not None) else 0.0
```

---

### C2. Aucune commande — y compris l'arrêt d'urgence — n'atteint jamais le robot

**Fichier :** `axis_os/hal/drivers/unitree_g1/g1_dds_actuator.py:236`

`G1DDSHub.publish()` écrit le message `LowCmd_` sur le bus DDS. C'est le seul
endroit du driver où quoi que ce soit part physiquement vers le robot.

`grep -rn "publish" --include=*.py` sur tout le repo : **aucun appelant.** Ni
dans `send_command()`, ni dans `emergency_stop()`, ni dans `UnitreeG1HAL`, ni
dans les tests.

`send_motor_command()` ne fait qu'écrire dans un buffer partagé en mémoire. Sans
`publish()`, ce buffer n'est jamais transmis.

**Scénario concret.** C'est le plus grave des deux sens :

- *Au mieux*, le robot ne bouge jamais et vous passez des heures à déboguer le
  réseau DDS, le CRC, le `mode_machine` — alors que le message n'est simplement
  jamais envoyé.
- *Au pire*, vous ajoutez l'appel `publish()` dans `send_command()` (le réflexe
  naturel) et vous oubliez `emergency_stop()`. Le robot exécute alors les
  commandes normales, mais **l'arrêt d'urgence écrit `kp=0, kd=0, tau=0` dans un
  buffer qui n'est jamais transmis**. Le robot continue à exécuter sa dernière
  commande publiée, indéfiniment, pendant que tout le système logiciel affiche
  « E-STOP déclenché ». C'est exactement le mode de défaillance qu'un e-stop
  existe pour empêcher.

**Correction.** `emergency_stop()` doit publier immédiatement et
inconditionnellement — c'est le seul chemin du système qui ne doit jamais
dépendre d'un appel groupé ultérieur. Pour les commandes normales, l'envoi
groupé après remplissage des 23 joints est le bon design (le protocole `LowCmd_`
transporte tous les moteurs d'un coup) : ajoutez un `hub.publish()` en fin de
`UnitreeG1HAL.send_joint_commands()`, pas dans `send_command()` individuel.

---

### C3. En mode couple, 9 articulations bougent dans le mauvais sens **[vérifié]**

**Fichiers :** `axis_os/hal/core/kinematic_normalizer.py:82-99` (commande) et
`:108` (feedback)

`command_to_raw()` ne convertit que la position et la vitesse, et seulement dans
les modes `POSITION`/`IMPEDANCE`/`VELOCITY`. Le couple est recopié tel quel, sans
inversion de signe. Mais `feedback_to_normalized()` fait bien
`torque = feedback.torque * entry.sign`.

Vérification exécutée sur `right_knee_joint` (sign = -1.0) :

```
C)  couple normalisé +20 Nm  -> raw +20.0 Nm   (attendu : -20.0)
C2) feedback raw +20 Nm      -> normalisé -20.0 Nm
```

L'asymétrie est nette : la lecture est normalisée, l'écriture ne l'est pas.
9 articulations sont concernées — `right_hip_roll/yaw/pitch`, `right_knee`,
`right_ankle_pitch/roll`, `right_shoulder_roll/yaw`, `right_wrist_roll` — dont
`right_hip_pitch_joint`, annoté « INVERSION G1 CRITIQUE » dans votre propre
table, et `right_knee_joint`, annoté « bug historique ».

**Scénario concret.** Une app de stabilisation commande +30 Nm de flexion sur les
deux genoux pour amortir un choc. Le genou gauche fléchit. Le genou droit
**étend** à 30 Nm. Le robot part en vrille latérale. Pire : le feedback renvoie
« -30 Nm » à l'app, qui en conclut correctement que le genou droit s'étend et
corrige en commandant encore plus de flexion — qui étend encore plus. Boucle
divergente.

Le même raccourci s'applique au mode `CURRENT` (jamais converti non plus), et le
courant moteur est proportionnel au couple : même inversion.

**Correction.** Dans `command_to_raw()`, traiter couple et courant comme le
feedback les traite déjà :

```python
entry = self._get(joint_id)
if command.torque is not None:
    raw.torque = command.torque * entry.sign
if command.current is not None:
    raw.current = command.current * entry.sign
```

Ajoutez un test de propriété aller-retour (`normalized → raw → normalized`
doit être l'identité, sur les 23 joints et les 5 modes) : il aurait attrapé ça
tout seul, et attrapera la prochaine asymétrie du même genre.

---

### C4. `on_start()` s'exécute hors sandbox, avec accès HAL direct **[vérifié]**

**Fichier :** `axis_os/store/runtime/play_store.py:236-248`

`run_mission()` isole correctement l'app (`SandboxRunner` + `HALBridgeClient`,
quotas CPU/RAM/timeout). Mais `start_app()` appelle :

```python
ctx = self._make_ctx(entry.manifest)
ready = entry.app.on_start(ctx)
```

directement dans le process parent, avec un vrai `RobotContext` — accès HAL
complet, aucun quota, aucun timeout, aucune isolation mémoire. Idem pour
`stop_app()`/`on_stop()`.

Ce n'est pas théorique : `axis_os/apps/package_sorting/app.py:120` appelle
`ctx.move_arms(_cmds(HOME))` **dans `on_start()`**. Le robot bouge physiquement
pendant le hook non sandboxé.

**Scénario concret.** Une app tierce met sa logique dans `on_start()` : elle a
accès direct au HAL, peut bloquer le process principal indéfiniment (aucun
timeout), consommer toute la RAM, et bouger les actionneurs sans passer par le
pont IPC. Toute l'architecture d'isolation documentée dans
`docs/sandboxing_hardening_notes.md` est contournée par un hook que *chaque*
app possède.

**Correction.** Passer `on_start`/`on_stop` par le même `SandboxRunner` +
`HALBridgeServer` que `run_mission`. Le pont existe déjà et expose la bonne
surface d'API — c'est du câblage, pas de la conception. Si vous voulez garder
`on_start` synchrone et rapide, mettez-lui au minimum un quota dédié et un
timeout dur.

---

### C5. Le couple réellement appliqué n'est borné par rien

**Fichiers :** `axis_os/hal/drivers/unitree_g1/g1_dds_actuator.py:103-121, 329-330`,
`axis_os/hal/safety/limits.py:217-225, 254`

Le guard vérifie `command.torque`. Mais en contrôle de position, le couple
physique n'est pas `command.torque` — c'est ce que produit le PD embarqué du
moteur :

```
τ_réel ≈ kp × (q_consigne − q_mesuré) + kd × (dq_consigne − dq_mesuré) + τ_ff
```

Avec `kp=100` sur le genou (valeur par défaut de l'exemple officiel, ligne 109),
un saut de consigne parfaitement légal de 1,5 rad — bien à l'intérieur de
`[-0,087 ; 2,88]` — commande ~150 Nm, au-dessus du `torque_max` de 139 Nm issu
de l'URDF. Le guard a validé la commande : la position est dans les limites et
`command.torque` vaut `None`.

Le garde-fou prévu pour ça — la limite d'accélération — **est intégralement mort
en production** :

```
grep accel_max / motion_tracker (hors tests) :
  - aucun appelant ne passe motion_tracker= à JointSafetyGuard
  - aucun code ne renseigne jamais limits.accel_max (URDF ne le fournit pas)
```

`limits.py:254` exige les deux. Les deux sont `None` partout. La règle
`ACCELERATION_LIMIT` ne peut structurellement jamais se déclencher — alors que
`robot.py:249-257` documente explicitement « SafetyGuard (rate limiting avant
envoi) » comme consommateur du tracker.

**Scénario concret.** Une app envoie une consigne de position légale mais
éloignée de la position courante (reprise après pause, changement de phase de
mission, sortie d'un modèle VLA). Le moteur saute à pleine puissance vers la
cible. Sur un bras : impact. Sur une jambe en appui : le robot se projette.

**Correction, en trois temps :**
1. Brancher le tracker : `JointSafetyGuard(..., motion_tracker=hal.get_motion_tracker())`
   dans `G1SimulatedActuator.__init__` et `G1DDSActuator.__init__`.
2. Renseigner `accel_max` par joint. L'URDF ne le fournit pas — c'est à définir
   dans une table AXIS-OS, au même titre que `current_max`/`temperature_max` qui
   sont déjà dans ce cas (voir `urdf_loader.py:132-137`).
3. Ajouter une borne explicite sur le delta de position par cycle
   (`|q_consigne − q_mesuré| ≤ τ_max / kp`), qui borne directement le couple PD.
   C'est le seul contrôle qui attaque la cause plutôt que le symptôme.

---

## 2. ÉLEVÉ — sécurité réelle, sans conséquence physique immédiate

### E1. La signature des apps est opt-in : aucune protection contre une app malveillante

`play_store.py:166` — `if manifest.code_signature is not None:`. Les deux champs
valent `None` par défaut. Une app non signée s'installe sans aucune vérification.

Le CHANGELOG présente la signature comme comblant le trou « rien n'empêchait un
fichier .py quelconque de se faire passer pour une app légitime ». Ce n'est pas
ce qui est implémenté : le mécanisme détecte la **modification accidentelle
d'apps qui ont choisi de se signer**. Un attaquant omet simplement le champ.

Second point : `inspect.getsource(type(app))` (ligne 170) ne couvre que le texte
de la classe elle-même — ni son module, ni ses imports, ni ses classes de base.
Tout le code utile peut vivre dans une fonction module-level et la signature reste
valide.

**Correction.** Signature obligatoire dès qu'un vrai Play Store s'ouvre à des
développeurs externes (un flag `require_signed_apps=True` sur `PlayStoreRuntime`,
`False` par défaut aujourd'hui, `True` avant toute ouverture). Et signer le
module entier (`inspect.getsourcefile` + hash du fichier), pas la classe. En
attendant, corriger la formulation du CHANGELOG : elle promet plus que ce qui
est en place.

### E2. `_apply_velocity_cap` n'est pas un plafond et mute les objets de l'appelant **[vérifié]**

`app_base.py:123-127` fait `cmd.velocity *= ratio` — une multiplication en place
sur l'objet de l'app, pas un plafond. Exécuté :

```
max_velocity_ratio = 0.8
appel 1: velocity = 0.8     appel 4: velocity = 0.4096
appel 2: velocity = 0.64    appel 5: velocity = 0.3277
```

Une app qui construit son dict de commandes une fois et le renvoie en boucle
(pattern très courant) voit sa vitesse décroître géométriquement. Après
20 itérations : 1 % de la consigne. Le mouvement devient silencieusement faux.

La direction est heureusement sûre (ça ralentit), mais c'est du sable dans les
rouages : un développeur d'app verra son robot « traîner » sans comprendre.

**Correction.** Ne pas muter l'entrée, et faire un vrai plafond :

```python
def _apply_velocity_cap(self, commands):
    return {jid: replace(cmd, velocity=(
                math.copysign(min(abs(cmd.velocity), v_max * ratio), cmd.velocity)
                if cmd.velocity is not None else None))
            for jid, cmd in commands.items()}
```

### E3. Le tracker enregistre les commandes rejetées — et le commentaire dit le contraire **[vérifié]**

`robot.py:229-234`. Le commentaire affirme : *« reflète ce qui a effectivement été
transmis au HAL, pas une commande hypothétique qui aurait pu être rejetée »*. Le
code appelle `record_command()` inconditionnellement, sans regarder `status`.

Exécuté : une commande `position=99.0` rejetée par le guard (`ERROR_LIMIT`) est
quand même enregistrée comme dernière commande du joint.

Conséquence réelle une fois C5 corrigé : le calcul d'accélération prendra une
commande refusée comme référence. Soit il déclenchera un faux positif sur la
commande légitime suivante, soit il masquera une vraie violation. Un garde-fou
qui raisonne sur des données falsifiées est pire que pas de garde-fou.

**Correction :** `if status == HALStatus.OK: self._motion_tracker.record_command(...)`.

### E4. `check_feedback_health()` n'est appelée nulle part en production

`limits.py:270` détecte `TORQUE_OVERLOAD` (couple mesuré > 110 % du max) et les
feedbacks non-finis. Seuls appelants : `test/test_safety_v2.py`. Aucun en
production.

Le watchdog fait un contrôle voisin (`_check_mechanical`) mais le classe
**CRITICAL**, et seul **FATAL** déclenche un e-stop. Un moteur qui force contre
un obstacle — un bras coincé sur un membre humain — produit donc une alerte
CRITICAL et rien d'autre : le robot continue de pousser.

**Correction.** Appeler `check_feedback_health()` dans `get_feedback()` ou dans
la boucle du watchdog, et décider explicitement si `TORQUE_OVERLOAD` confirmé sur
N échantillons doit passer FATAL. Pour un humanoïde en environnement partagé,
mon avis : oui.

### E5. `UnitreeG1HAL` n'instancie jamais `G1DDSActuator`

`robot.py:190` : `act = G1SimulatedActuator(jdef)`, en dur, sans option. La
docstring du module (ligne 6) annonce « En production : G1DDSActuator ». Il
n'existe aucun chemin de code pour construire un HAL matériel. Le driver réel
est du code mort.

**Correction :** `UnitreeG1HAL(robot_id, backend="sim"|"dds", network_interface=...)`,
qui construit le hub une fois et le passe à chaque actionneur.

### E6. Course sur le buffer `LowCmd_` partagé et sur `_low_state`

`g1_dds_actuator.py:214-242`. `G1DDSHub` est explicitement partagé par les
23 actionneurs. Aucun verrou. Trois écrivains concurrents possibles : le thread
de mission, le thread du watchdog (qui appelle `emergency_stop_all()` →
`send_motor_command()` sur les 23 joints), et le callback DDS (`_on_low_state`).

Le cas qui fait mal : le watchdog déclenche un e-stop pendant que le thread de
mission remplit le buffer. Les écritures s'entrelacent, `publish()` calcule le
CRC sur un message moitié « arrêt » moitié « mouvement », et le firmware reçoit
un état mixte parfaitement valide au sens CRC.

**Correction :** un `threading.RLock` dans `G1DDSHub`, pris par
`send_motor_command()`, `publish()`, `read_motor_state()` et `_on_low_state()`.
Le coût est négligeable devant la latence DDS.

### E7. `_ensure_low_level_mode()` : boucle sans borne, code de retour ignoré

`g1_dds_actuator.py:145-149` :

```python
status, result = msc.CheckMode()
while result.get("name"):
    msc.ReleaseMode()
    status, result = msc.CheckMode()
    time.sleep(1)
```

`status` n'est jamais consulté. Si `CheckMode()` échoue (robot injoignable,
service indisponible), `result` revient probablement vide → la boucle sort
immédiatement → `initialize()` continue et publie des commandes bas niveau
**alors que le mode sport intégré peut être toujours actif**. C'est très
exactement le point n°1 de votre propre section « Points de sécurité
obligatoires » : conflit de commandes direct sur les moteurs.

Symétriquement, si `ReleaseMode()` ne prend jamais effet, la boucle tourne
indéfiniment : `initialize()` ne rend jamais la main, sans timeout ni message.

**Correction :** vérifier `status`, borner la boucle (10 tentatives / 15 s), et
retourner `HALStatus.ERROR_COMM` si le mode sport n'est pas confirmé désactivé.
Ne jamais continuer sur un doute ici.

### E8. Le serveur du pont HAL meurt sur un message malformé

`hal_bridge.py:110` : `response = self._handle(HALCommand(**raw))`. La
construction `HALCommand(**raw)` est **hors** de tout `try` — le `try/except` est
à l'intérieur de `_handle`. `raw` vient du process enfant, c'est-à-dire du côté
non fiable de la frontière de sécurité.

Une app envoie `{"action": "x"}` ou `{"foo": 1}` → `TypeError` non rattrapée →
le thread `hal_bridge_server` (daemon) meurt silencieusement. Plus aucune
commande n'est servie ; l'app voit des `TimeoutError` ; aucune alerte ne remonte.

**Correction :** valider le dict avant construction (clés attendues, types), tout
envelopper dans le `try`, et logguer la mort du thread comme une anomalie de
sécurité.

### E9. Sans `psutil`, les quotas CPU/RAM sont silencieusement désactivés

`sandbox.py:93-96` et `288-289` : si l'import échoue, `_monitor_loop()` retourne
immédiatement. Aucun avertissement, aucune trace. `SandboxRunner` continue de
rapporter des exécutions réussies en n'appliquant plus rien (le timeout, lui,
reste couvert par le `proc.join()` de `run()`).

`psutil` est bien dans `requirements.txt`, donc le cas nominal va. Mais une
installation partielle produit un sandbox qui ment sur ce qu'il garantit.

**Correction :** log d'avertissement bruyant au démarrage, et un
`SandboxRunner(strict=True)` qui refuse de démarrer sans `psutil` — à utiliser
dès qu'une app non vérifiée tourne.

### E10. L'e-stop du Supervisor est invisible pour le reste du système

`supervisor.py:251` appelle `hal.emergency_stop_all()` sans jamais positionner
`WatchdogSystem._estop_fired`. Or `PlayStoreRuntime.run_mission()` (ligne 264) et
`RobotContext._require_no_estop()` (`app_base.py:117`) lisent tous les deux
`watchdog.estop_fired`.

Après un e-stop déclenché par le Supervisor — donc précisément quand le watchdog
est mort ou bloqué — le Play Store accepte encore de lancer des missions et les
apps croient le robot disponible. Les actionneurs, eux, refusent (`ERROR_FAULT`),
donc le matériel est protégé : la dernière couche tient. Mais les trois couches
au-dessus affichent un état faux.

**Correction :** un état e-stop unique et partagé (porté par le HAL, pas par le
watchdog), que Supervisor et Watchdog positionnent tous deux et que tout le monde
lit.

---

## 3. MOYEN — dette technique et incohérences à traiter sans urgence

| # | Point | Fichier |
|---|---|---|
| M1 | **7 permissions déclarées, jamais vérifiées** : `SENSORS_FORCE_TORQUE`, `SENSORS_CAMERA`, `SENSORS_MICROPHONE`, `EMERGENCY_STOP`, `FLEET_BROADCAST`, `NETWORK_CLOUD`, `NETWORK_LOCAL`. Zéro `_require()` pour chacune. Les deux permissions réseau sont les plus gênantes : rien n'empêche une app d'ouvrir une socket, le sandbox `multiprocessing` ne filtrant pas le réseau (documenté dans `docs/sandboxing_hardening_notes.md`). Soit les retirer de l'enum, soit les appliquer. | `manifest.py:17-30` |
| M2 | **Tous les modèles tarifaires sont facturés par mission.** `_bill()` ne regarde pas `pricing_model` sauf pour exclure `FREE`. Une app `ONE_TIME` facture son prix complet à **chaque** mission ; une app `SUBSCRIPTION` facture le mois entier à chaque mission. Bug financier direct côté client. | `play_store.py:425-433` |
| M3 | **La facturation dépend du `success` auto-déclaré par l'app.** `MissionResult` vient du process enfant ; l'app décide donc si le client est facturé. En `PAY_PER_MISSION`, un développeur peut toujours retourner `success=True`. | `play_store.py:371-377` |
| M4 | **`current_max = 15 A` identique pour les 23 joints** (défaut URDF, hanche comme poignet), et le courant n'est jamais vérifié côté feedback — seulement `command.current`, non-`None` uniquement en mode `CURRENT`. La protection en courant est décorative. | `urdf_loader.py:166` |
| M5 | **Capteurs fabriqués.** `G1SimulatedActuator.get_feedback()` invente `current = torque / 10`. `WebotsBackend` renvoie une température constante (`STUB_TEMP`) même en mode production. Le watchdog surveille donc une température qui ne bougera jamais. | `robot.py:119`, `webots/backend.py` |
| M6 | **`except Exception: pass` à la liaison des moteurs Webots** (6 occurrences). Un joint qui ne se lie pas disparaît silencieusement de `_motors` ; `connect()` retourne quand même `True`. Les commandes suivantes sur ce joint échouent en silence → exécution partielle d'un mouvement coordonné. Sur un bipède, c'est une chute. | `webots/backend.py:68,72,111,137,165,172` |
| M7 | **`stiffness`/`damping` jamais validés** — `NaN` et `1e9` passent le guard **[vérifié]** — et sont perdus à la traversée du pont HAL (`_serialize`/`_deserialize_commands` ne les transportent pas, `current` non plus). Le mode `IMPEDANCE` est donc annoncé mais non fonctionnel côté sandbox, et `G1DDSActuator` ignore de toute façon ces champs au profit de `_DEFAULT_KP/KD`. | `limits.py:180-183`, `hal_bridge.py:171-179, 253-258` |
| M8 | **L'offset est appliqué aux vitesses.** `command_to_raw()` passe la vitesse par `normalized_to_raw()`, qui ajoute `offset_rad`. Un offset est un décalage de position : l'ajouter à une vitesse injecte un biais constant — commander « vitesse 0 » produirait `offset_rad ≠ 0`, donc un joint qui continue de bouger. Latent aujourd'hui (les 23 offsets du G1 sont à 0,0), mais c'est une mine pour le premier robot qui en aura un. Le couple, lui, est correctement traité (`* sign` seul) dans le feedback — la bonne logique existe déjà à côté. | `kinematic_normalizer.py:94-98` |
| M9 | **L'e-stop n'est jamais réarmable.** `_estop_fired` passe à `True` et rien ne le remet à `False` ; aucun `reset_estop()`. Après un e-stop, le seul recours est de redémarrer le process. Défendable comme choix de sécurité, mais alors à documenter comme tel — aujourd'hui ça ressemble à un oubli. | `watchdog/system.py:505-508` |
| M10 | **Le watchdog ne surveille jamais la position.** `_check_mechanical()` couvre couple et vitesse. Un feedback de position hors `[pos_min, pos_max]` — butée mécanique franchie, encodeur qui dérive — n'est signalé par rien. | `watchdog/system.py:410-448` |
| M11 | **Une app ne peut pas déclencher un e-stop.** `RobotContext` n'expose pas `emergency_stop()` (constat déjà noté dans `warehouse_ops/app.py:30`). Une app de patrouille qui détecte un humain dans la trajectoire n'a aucun moyen d'arrêter le robot. La permission `EMERGENCY_STOP` existe pourtant déjà dans l'enum. | `app_base.py` |
| M12 | `requirments.txt` (faute de frappe) traîne à côté de `requirements.txt`, avec un contenu périmé. | racine |

---

## 4. Tests qui donnent une fausse impression de couverture

**`test/test_g1_dds_actuator.py` — 19 tests verts, dont deux qui ne prouvent pas
ce qu'ils annoncent.**

La section 4 s'intitule « CRITIQUE : une commande dangereuse n'atteint JAMAIS le
hub DDS » et la section 5 teste l'arrêt d'urgence. Les deux utilisent un
`MagicMock` de `G1DDSHub` et assertent sur les kwargs de `send_motor_command()`.

Or `send_motor_command()` n'envoie rien — elle écrit dans un buffer local (C2).
Le test vérifie donc que l'e-stop a bien *écrit* `kp=0.0` dans un message qui, en
l'état du code, ne part jamais. Aucune assertion sur `publish()`. Le test serait
tout aussi vert si `publish()` n'existait pas du tout — ce qui, fonctionnellement,
est le cas aujourd'hui.

De même, la section 4 ne teste la position hors limites qu'en mode `POSITION` —
le mode où le guard la vérifie. Le contournement C1 passe entre les mailles.

**Correctifs de test à ajouter :**
- Un mock de hub qui assert `publish()` appelé après chaque `send_command()` et
  **immédiatement** après `emergency_stop()`.
- Un test paramétré sur les 5 `ControlMode` × (position valide / hors limites) :
  il aurait attrapé C1 seul.
- Un test aller-retour du normalizer sur les 23 joints × 5 modes : il aurait
  attrapé C3 seul.
- Un test de commande rejetée qui vérifie que le `MotionStateTracker` **n'a pas**
  été mis à jour (E3).

---

## 5. Ce qui est solide — vérifié, pas de régression

Les corrections des audits précédents tiennent, et bien :

- **URDF comme source de vérité.** `urdf_loader.py` est rigoureux : refus strict
  d'un joint actionné sans `<limit>`, refus d'un type non supporté plutôt
  qu'ignoré, détection explicite des `.xacro` non développés, distinction
  `None` / `0.0` dans `_parse_float_attr`. Les 23 joints se chargent, le
  `torque_max` du genou est bien à 139 Nm. Aucune régression vers le codage
  manuel.
- **Verrou de mission.** `acquire(blocking=False)` + `try/finally` — refus
  immédiat et explicite, relâchement garanti même sur exception. Correct.
- **Prix minimum viable.** Le seuil est bien recalculé depuis `COMMISSION_RATE`,
  pas codé en dur, et `FREE` est correctement exclu.
- **Chaînage de hash des logs.** Bien fait, rétrocompatible, `verify_chain()`
  signale honnêtement les logs pré-chaînage au lieu de planter.
- **Validation de la table de conventions avant tout mouvement**
  (`robot.py:174-187`), et `validate_limits()` à la construction du guard : les
  deux échouent bruyamment, ce qui est le bon réflexe.
- **Aucun secret en dur** dans tout le repo (grep sur `api_key|secret|token|
  password|private_key|ghp_|sk-|AKIA`). Les clés des tests sont générées à la
  volée.
- **Les 5 documents de `docs/` sont honnêtes** et utiles. Le statut « NON CODÉ »
  est explicite, les conditions de reprise sont formulées, et les justifications
  (Fleet Manager avant sécurité réseau, sandboxing renforcé après signature des
  apps) tiennent la route. Je n'ai rien à redire sur ces choix — le séquencement
  est le bon. Je n'ai donc rien traité de ces quatre sujets comme un oubli.

---

## 6. Ordre de traitement suggéré

Avant toute mise sous tension d'un G1 physique, dans cet ordre :

1. **C2** (`publish()`, e-stop en priorité) — sans ça, rien ne fonctionne, et le
   mode de panne est piégeux.
2. **C1** (validation de position tous modes) — une ligne, gros retour.
3. **C3** (normalisation du couple/courant) — quelques lignes + test aller-retour.
4. **E7** (mode sport confirmé désactivé) et **E6** (verrou du hub) — les deux
   sont dans le chemin d'initialisation matérielle.
5. **C5** (borne sur le delta de position, tracker branché) — le plus gros
   chantier des cinq, mais c'est celui qui protège contre les commandes
   *légales* mais brutales, y compris celles d'un modèle VLA.

Ensuite seulement : **C4** (sandboxing d'`on_start`), **E3**, **E4**, **E1**, et
la liste MOYEN.

Un point de méthode, pour finir : trois des cinq problèmes critiques sont des
**incohérences entre deux moitiés du même mécanisme** — position validée d'un
côté / transmise de l'autre, couple normalisé en lecture / pas en écriture,
commande écrite dans un buffer / buffer jamais publié. Aucun n'est une erreur de
conception : ce sont des ruptures de symétrie. Des tests de propriété
(aller-retour, invariants sur tous les joints × tous les modes) les attrapent
bien mieux que des tests par cas, et c'est probablement le meilleur
investissement de test que vous puissiez faire sur ce projet.
