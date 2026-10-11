# Validation de la commande vocale — 2026-09-23

Mesures réelles contre Isaac Sim 6.1.0, G1 23 DOF, app sandboxée via le
Play Store. Reproductible via `sim/validation/test_isaac_a2_voice.py`.

---

## 1. Résultat

**Le robot obéit à la voix, et ignore ce qu'il ne comprend pas.**

Boucle acoustique **fermée** : les phrases sont synthétisées
(`facebook/mms-tts-fra`), transcrites par Whisper, converties en
intention, puis exécutées. L'audio traverse réellement le modèle.

| | |
|---|---|
| « va vers le carton. » | comprise → `aller_vers_colis` |
| « Il fait beau aujourd'hui. » | **ignorée** — aucune intention |
| « stop » → transcrit **« So. »** | **non comprise** (voir §3) |
| Mission | **réussie en 13.5 s**, arrivée à 1.03 m |
| Déplacement | x = 1.36 m sur commande vocale |
| Garde-fou | 0 consigne refusée, 0 chute |

---

## 2. Quatre étages entre une phrase et un moteur

```
voix -> Whisper            : rend du TEXTE, jamais une action
     -> registre d'intentions : doit correspondre à une intention
                                DÉCLARÉE par l'app, sinon rien
     -> l'app (code, permissions) : décide quoi en faire
     -> JointSafetyGuard     : vérifie chaque commande
```

Une phrase imprévue ne produit **aucun** mouvement — non parce qu'on l'a
interdit, mais parce qu'aucun chemin n'existe. Vérifié : deux phrases sur
trois n'ont rien déclenché.

L'app **déclare** son vocabulaire au démarrage. Installer une app ajoute
des mots que le robot comprend, sans réentraîner aucun modèle.

---

## 3. « stop » n'a pas été compris — à traiter

Whisper a transcrit le « stop » synthétisé en « So. ». Aucune intention
n'a correspondu, donc **l'ordre d'arrêt a été ignoré**. La mission a
réussi parce que le robot arrivait de toute façon.

C'est le résultat le plus important de ce test, et il est négatif.

- Un mot isolé d'une syllabe est ce qu'un modèle de transcription
  reconnaît le plus mal — peu de contexte, durée très courte.
- C'est précisément le mot qui doit fonctionner à tous les coups.

**Conséquences à retenir :**
- préférer des formulations plus longues (« arrête-toi », « stoppe le
  robot ») et enrichir les synonymes ;
- ne jamais faire reposer une fonction de sécurité sur la
  reconnaissance vocale : **l'arrêt d'urgence matériel reste le seul
  moyen fiable** ;
- un mot-clé d'arrêt mériterait un détecteur dédié plutôt qu'une
  transcription complète.

---

## 4. Le Play Store a refusé l'app — et il avait raison

Premier essai : installation **refusée**, « capacité requise absente :
SPEECH_IN ». Le robot ne déclarait aucun micro, et une app qui en a
besoin ne doit pas s'installer sur un robot qui n'en a pas.

Corrigé en déclarant `MICROPHONE` et `SPEAKER` sur le G1, selon la
convention déjà en place (les capacités décrivent le robot, pas ce
qu'AXIS-OS pilote). **À vérifier sur l'exemplaire réel** : une capacité
déclarée à tort produit l'erreur symétrique — une app installée qui ne
peut pas fonctionner.

**Manque restant** : le HAL n'expose pas `get_microphone()`. La capacité
est déclarée dans le driver, le capteur injecté dans le service : les
deux peuvent diverger. Un capteur devrait être attaché au HAL, qui en
déduirait sa capacité.

---

## 5. Le robot s'arrête pendant qu'il écoute

`stale = 2` : pendant les deux secondes d'écoute, la consigne de vitesse
a expiré et le robot s'est immobilisé de lui-même, en équilibre.

Comportement voulu : il n'avance pas à l'aveugle pendant qu'il écoute.
Et l'approche est découpée en salves de 3 s entre deux écoutes — un
robot qui n'écoute plus pendant qu'il agit ne peut pas être arrêté à la
voix.

---

## 6. Limites

- **Voix synthétique**, pas une voix humaine enregistrée : ni accent, ni
  bruit ambiant, ni distance au micro, ni réverbération.
- **Pas de détection d'activité vocale** : le service transcrit une
  durée fixe. Un robot qui écoute en continu doit savoir *quand*
  quelqu'un parle.
- **Une seule langue testée** (français), un seul locuteur.
