"""
AXIS-OS — Rapports de plantage : quand une app plante, dépasse un quota ou
se bloque (délai dépassé), l'OS garde un rapport, que l'administrateur lit
(CrashReportStore.reports) ; aucune app n'y a accès (pas d'action du pont).

Un rapport ne contient ni donnée personnelle ni secret : l'app (app_id,
version), le hook, la cause (SandboxErrorKind), le quota enfreint, la durée,
l'heure, et le TYPE de l'exception, jamais son message ni la pile, les
paramètres de la mission ou le journal de l'app (ils peuvent contenir ce
que l'app a vu ou entendu).
"""
from __future__ import annotations

import re
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional

REPORTS_MAX = 200
_TYPE_EXCEPTION = re.compile(r"\b([A-Z][A-Za-z0-9_]{2,60}(?:Error|Exception|Interrupt|Exit))\b")


@dataclass(frozen=True)
class CrashReport:
    app_id: str
    version: str
    hook: str
    kind: str
    exception_type: Optional[str]
    quota_rule: Optional[str]
    quota_value: Optional[float]
    quota_limit: Optional[float]
    duration_s: float
    timestamp: float


class CrashReportStore:
    """Rapports de plantage, bornés (REPORTS_MAX, les plus anciens partent)."""

    def __init__(self):
        self._verrou = threading.Lock()
        self._rapports: Deque[CrashReport] = deque(maxlen=REPORTS_MAX)

    def record(self, app_id: str, version: str, hook: str, sandbox_result) -> CrashReport:
        kind = getattr(sandbox_result.error_kind, "name", None) or "UNKNOWN"
        m = _TYPE_EXCEPTION.search(str(sandbox_result.error or ""))
        v = sandbox_result.violation
        r = CrashReport(app_id, version, hook, kind, m.group(1) if m else None,
                        getattr(v, "rule", None), getattr(v, "value", None),
                        getattr(v, "limit", None), round(float(sandbox_result.duration_s), 3),
                        time.time())
        with self._verrou:
            self._rapports.append(r)
        return r

    def reports(self, app_id: Optional[str] = None) -> List[CrashReport]:
        """Pour l'administrateur : tous les rapports, ou ceux d'une app."""
        with self._verrou:
            return [r for r in self._rapports if app_id is None or r.app_id == app_id]
