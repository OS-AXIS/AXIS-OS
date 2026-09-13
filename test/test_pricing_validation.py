"""
AXIS-OS — Tests validation prix / commission
================================================
Valide le correctif ajouté à AppManifest.validate() suite à un audit :
avec un base_price_eur trop bas (ex: 0.01€), l'arrondi à 2 décimales
de store_price_eur pouvait ramener platform_commission_eur à 0.00€ —
la plateforme ne touchant alors littéralement rien sur une app
pourtant déclarée payante.
"""

import sys
sys.path.insert(0, ".")

from axis_os.store.sdk.manifest import AppManifest, AppCategory, PricingModel

FAIL = 0


def check(description: str, condition: bool) -> None:
    global FAIL
    status = "✓" if condition else "✗"
    print(f"  [{status}] {description}")
    if not condition:
        FAIL += 1


def section(title: str) -> None:
    print(f"\n{'=' * 66}\n  {title}\n{'=' * 66}")


def make_manifest(price: float, model=PricingModel.PAY_PER_MISSION):
    return AppManifest(
        app_id="com.test.pricing", name="T", version="1.0.0", author="T",
        author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
        pricing_model=model, base_price_eur=price,
    )


# ══════════════════════════════════════════════════════════════════

section("1 — Le cas bug exact (0.01€) est maintenant rejeté")

m_bug = make_manifest(0.01)
check("platform_commission_eur est bien 0.0€ pour ce prix (confirme le bug)",
      m_bug.platform_commission_eur == 0.0)
errors = m_bug.validate()
check("validate() détecte le problème (1 erreur)", len(errors) == 1)
check("Le message mentionne explicitement 'trop bas'",
      len(errors) == 1 and "trop bas" in errors[0])

# ══════════════════════════════════════════════════════════════════

section("2 — Le seuil minimum valide (0.02€) passe sans erreur")

m_min = make_manifest(0.02)
check("platform_commission_eur > 0 à ce seuil",
      m_min.platform_commission_eur > 0)
check("validate() ne remonte aucune erreur", m_min.validate() == [])

# ══════════════════════════════════════════════════════════════════

section("3 — Les prix déjà utilisés par les apps de référence restent valides")

for price in (1.40, 49.00, 2.00, 70.00):
    m = make_manifest(price)
    errs = m.validate()
    check(f"base_price_eur={price}€ (déjà en usage) -> aucune erreur",
          errs == [])

# ══════════════════════════════════════════════════════════════════

section("4 — Les apps FREE ne sont jamais affectées par cette règle")

m_free = make_manifest(0.0, model=PricingModel.FREE)
check("platform_commission_eur == 0.0€ pour une app FREE (normal, pas un bug)",
      m_free.platform_commission_eur == 0.0)
check("validate() ne signale AUCUNE erreur pour une app FREE à 0€",
      m_free.validate() == [])

# ══════════════════════════════════════════════════════════════════

section("5 — Cohérence : la commission reste toujours ~30% une fois validée")

for price in (0.02, 0.05, 0.10, 1.0, 10.0, 100.0):
    m = make_manifest(price)
    if m.validate() == []:  # ne teste que les prix qui passent la validation
        rate = m.platform_commission_eur / m.store_price_eur * 100
        check(f"base={price}€ -> taux de commission proche de 30% (obtenu {rate:.1f}%)",
              25.0 <= rate <= 35.0)  # tolérance large pour l'arrondi au centime

# ══════════════════════════════════════════════════════════════════

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)

if FAIL == 0:
    print("\n  ✓ Le bug de commission nulle sur micro-prix (0.01€) est détecté")
    print("  ✓ Le seuil minimum valide (0.02€) fonctionne correctement")
    print("  ✓ Aucune régression sur les prix déjà utilisés par les apps existantes")
    print("  ✓ Les apps gratuites ne sont jamais affectées par cette règle")

sys.exit(0 if FAIL == 0 else 1)
