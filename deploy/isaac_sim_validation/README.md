# Validation AXIS-OS contre Isaac Sim

Scripts de validation qui nécessitent un vrai Isaac Sim (conteneur
Docker officiel NVIDIA `nvcr.io/nvidia/isaac-sim:6.1.0`) — non
exécutables en CI classique, à lancer manuellement sur une instance
avec GPU. Voir `axis_os/hal/drivers/isaac_sim/bridge.py` pour le pont
réutilisable entre `UnitreeG1HAL` et Isaac Sim, et
`docs/isaac_lab_unitree_sim_status.md` pour l'historique complet des
diagnostics (VariantSet Physics, gains PD absents, thread-safety
PhysX).

Le dossier contient trois scripts, un par app de référence :

| Script | App testée de bout en bout |
|--------|----------------------------|
| `test_s5_full_stack.py` | `PackageSortingApp` (tri de colis) |
| `test_s5_security_patrol.py` | `SecurityPatrolApp` (ronde de sécurité) |
| `test_s5_warehouse_ops.py` | `WarehouseOpsApp` (opérations d'entrepôt) |

## Prérequis

```bash
docker cp ~/AXIS-OS/axis_os isim-isaac-sim-1:/isaac-sim/axis_os
docker cp ~/AXIS-OS/axis_os/hal/drivers/unitree_g1/urdf/g1_23dof_rev_1_0.urdf \
    isim-isaac-sim-1:/isaac-sim/g1_23dof_rev_1_0.urdf
docker cp ~/AXIS-OS/deploy/isaac_sim_validation/test_s5_full_stack.py \
    isim-isaac-sim-1:/isaac-sim/test_s5_full_stack.py
```

## test_s5_full_stack.py

Pile complète : `PlayStoreRuntime` → sandbox → `HALBridgeServer` →
HAL → Isaac Sim, avec `PackageSortingApp` (vraie app AXIS-OS, pas un
mock). Utilise `MainThreadSimPump` (voir `bridge.py`) — architecture
obligatoire dès qu'une app tourne dans le sandbox multiprocess de
`PlayStoreRuntime` :

```bash
docker exec -it isim-isaac-sim-1 timeout 200 /isaac-sim/python.sh /isaac-sim/test_s5_full_stack.py
```

**Pourquoi `MainThreadSimPump` est indispensable** : `World.step()`/
`apply_action()` d'Isaac Sim ne sont pas thread-safe et bloquent dans
du code C sans relâcher le GIL — appelés depuis le thread
`HALBridgeServer._loop()` (thread secondaire par construction, voir
`axis_os/store/runtime/hal_bridge.py`), ils gèlent le process entier,
y compris le thread principal en attente d'un `join(timeout=...)`.
Confirmé par test isolé : un `threading.Thread` qui appelle
`world.step()` bloque indéfiniment, sans qu'aucun timeout Python ne
permette de s'en sortir. La seule issue : que tous les appels
Isaac Sim aient lieu exclusivement sur le thread principal, et que ce
dernier devienne la boucle de simulation (`run_forever()`) pendant
que la mission AXIS-OS tourne dans un thread annexe
(`run_mission_in_thread()`).

Surveiller l'apparition de `ERROR_COMM` dans les statuts retournés —
signe qu'un `submit()` a dépassé son timeout de 5s (la pompe ne tourne
pas assez vite ou s'est arrêtée), pas un bug du pont lui-même.

## test_s5_security_patrol.py et test_s5_warehouse_ops.py

Même architecture que `test_s5_full_stack.py` (`MainThreadSimPump`, voir
la section précédente), contre les deux autres apps de référence :
`SecurityPatrolApp` et `WarehouseOpsApp`. Les mêmes prérequis valent ; on
copie en plus les deux scripts, puis on les lance de la même façon :

```bash
docker cp ~/AXIS-OS/deploy/isaac_sim_validation/test_s5_security_patrol.py \
    isim-isaac-sim-1:/isaac-sim/test_s5_security_patrol.py
docker cp ~/AXIS-OS/deploy/isaac_sim_validation/test_s5_warehouse_ops.py \
    isim-isaac-sim-1:/isaac-sim/test_s5_warehouse_ops.py
docker exec -it isim-isaac-sim-1 timeout 200 /isaac-sim/python.sh /isaac-sim/test_s5_security_patrol.py
docker exec -it isim-isaac-sim-1 timeout 200 /isaac-sim/python.sh /isaac-sim/test_s5_warehouse_ops.py
```

Résultats obtenus le 2026-09-19 (détail dans
`docs/isaac_lab_unitree_sim_status.md`, session 4) : `SecurityPatrolApp`
visite ses 7 points de passage et traite deux détections de personne,
`WarehouseOpsApp` scanne et manipule deux objets ; aucune violation de
sécurité dans les deux cas.
