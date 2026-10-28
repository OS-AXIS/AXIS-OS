"""
AXIS-OS — État du réseau (ctx.network_status) : « online » (une route vers
l'extérieur existe), « local_only » (une interface est active, sans route
vers l'extérieur), « offline » (aucune interface active hors boucle locale)
ou « unknown » (rien de lisible ici). Aucune connexion n'est ouverte pour le
savoir : on lit l'état des interfaces et la table de routage.
"""
from __future__ import annotations

import os
from typing import Optional

STATES = ("online", "local_only", "offline", "unknown")


class SystemNetworkProbe:
    """Lit /sys/class/net et /proc/net/route (Linux) ; « unknown » ailleurs.
    shortcut: « online » veut dire « route par défaut présente », pas
    « Internet joignable » ; à remplacer par un test du vrai service visé
    quand il y en aura un."""

    def __init__(self, sys_net: str = "/sys/class/net", route: str = "/proc/net/route"):
        self._sys_net, self._route = sys_net, route

    def _active(self, nom: str) -> bool:
        with open(os.path.join(self._sys_net, nom, "operstate")) as f:
            return f.read().strip() in ("up", "unknown")

    def status(self) -> str:
        try:
            actives = [n for n in os.listdir(self._sys_net) if n != "lo" and self._active(n)]
        except OSError:
            return "unknown"
        if not actives:
            return "offline"
        try:
            with open(self._route) as f:
                lignes = f.read().splitlines()[1:]
        except OSError:
            return "unknown"
        defaut = any(len(c) > 2 and c[1] == "00000000" and c[0] in actives
                     for c in (l.split() for l in lignes))
        return "online" if defaut else "local_only"


class FakeNetworkProbe:
    """État fixé par le test (`state`)."""

    def __init__(self, state: Optional[str] = "online"):
        self.state = state

    def status(self) -> str:
        return self.state if self.state in STATES else "unknown"
