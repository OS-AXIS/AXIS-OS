"""
Campagne de cartons variés : la prise ne doit pas marcher que sur le carton
de la scène T1 (24 × 20 × 20 cm, ~1 kg, même table, même place).

axis_os/testing/cartons.py tire des cartons au hasard, graine fixée, donc
rejouables (ici et sur le banc Isaac, lancer_t1.sh --cartons-varies) :
largeur 15 à 35 cm, profondeur 15 à 30, hauteur 10 à 30, masse 0,2 à
2,5 kg, frottement des mains 0,5 à 1,2, place sur la table dans et hors de
portée, lacet de -45 à 45° ; exprès, un sur dix trop large pour les mains,
un sur dix trop lourd pour les plafonds de couple. Chacun est essayé comme
sur le banc T1 (approche à la distance de prise, erreur de navigation
comprise, bras ouverts juste assez, prise, port, dépose sur la table),
avec le bruit, le biais et le gain des couples estimés du banc.

Chaque prise finit tenue et déposée, ou refusée, ou reposée, avec une
raison claire. JAMAIS : carton tombé, écrasé (consigne de serrage au-delà
de ses bornes, contact vu trop tard), plafond de couple dépassé (couple
vrai, à l'erreur d'estimation du banc près ; limite matérielle, toujours),
prise confirmée les mains vides, carton perdu sans que l'OS le voie. Le
moindre de ces cas fait échouer le test.

La repose aussi : chaque carton du balayage tenu est encore reposé à sa
place (poser() sans plateau) au lieu d'être déposé ; un carton qui tombe en
repose est un cas interdit, comme au port.

Le rapport montre les vraies limites du robot : réussite par plage, toutes
les raisons de refus, un balayage d'un paramètre à la fois autour du
carton T1, et l'audit des réglages choisis sur ce seul carton.
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.testing import cartons as K  # noqa: E402

FAIL = 0
GRAINE = 20261118
N = 120


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def dans(v, plage):
    return plage[0] - 1e-9 <= v <= plage[1] + 1e-9


def jamais(resultats, quoi, mots):
    """Aucun résultat dont une violation contient l'un des `mots`."""
    fautifs = [(x["carton"]["numero"], v) for x in resultats for v in K.violations(x)
               if any(m in v for m in mots)]
    check(f"Jamais {quoi}", not fautifs, "; ".join(f"n° {n} : {v}" for n, v in fautifs[:4]))


if __name__ == "__main__":
    t0 = time.perf_counter()
    section("1. Cartons tirés : rejouables, dans leurs plages, exprès hors des moyens")
    cartons = K.tirer_cartons(N, GRAINE)
    check(f"Même graine, mêmes {N} cartons ; autre graine, autres cartons",
          cartons == K.tirer_cartons(N, GRAINE) and cartons != K.tirer_cartons(N, GRAINE + 1))
    # Le tirage ne dépend que de random.Random(graine).random() : le banc Isaac
    # (une autre version de Python) tire les mêmes. Premier carton de la graine :
    premier = K.CartonTire(numero=0, largeur=0.3303, profondeur=0.2746, hauteur=0.1686,
                           masse_kg=0.305, frottement=0.513, lacet_deg=-10.68, retrait_m=0.1237,
                           decalage_m=-0.0306, erreur_x_m=-0.0134, erreur_y_m=0.0071,
                           erreur_cap_deg=-2.98)
    check("… le premier carton de la graine est celui attendu (rejouable ailleurs)",
          cartons[0] == premier, str(cartons[0]))
    larges = [c for c in cartons if c.expres == "trop large"]
    lourds = [c for c in cartons if c.expres == "trop lourd"]
    check(f"Exprès hors des moyens du robot : {len(larges)} trop larges ({100 * K.TROP_LARGE_M[0]:.0f} "
          f"à {100 * K.TROP_LARGE_M[1]:.0f} cm), {len(lourds)} trop lourds ({K.TROP_LOURD_KG[0]:g} à "
          f"{K.TROP_LOURD_KG[1]:g} kg)", len(larges) == len(lourds) == N // 10)
    check("Chaque grandeur dans sa plage", all(
        dans(c.largeur, K.TROP_LARGE_M if c.expres == "trop large" else K.LARGEUR_M)
        and dans(c.profondeur, K.PROFONDEUR_M) and dans(c.hauteur, K.HAUTEUR_M)
        and dans(c.masse_kg, K.TROP_LOURD_KG if c.expres == "trop lourd" else K.MASSE_KG)
        and dans(c.frottement, K.FROTTEMENT) and dans(c.lacet_deg, K.LACET_DEG)
        and dans(c.retrait_m, K.RETRAIT_M) and dans(c.decalage_m, K.DECALAGE_M)
        for c in cartons))

    balayes = K.balayage()
    tous = K.campagne(cartons + [c for _, _, c in balayes])
    res, bal = tous[:N], tous[N:]
    reposes = K.campagne([c for _, _, c in balayes], reposer=True)

    section(f"2. {N} cartons tirés : ce qui ne doit jamais arriver")
    jamais(res, "de carton tombé", ("tombé",))
    jamais(res, "de carton écrasé", ("écrasé",))
    jamais(res, "de plafond de couple dépassé", ("plafond de couple", "limite matérielle"))
    jamais(res, "de prise confirmée les mains vides, ni de carton perdu sans que l'OS le voie",
           ("mains vides", "sans que l'OS le voie"))
    jamais(res, "de carton laissé ailleurs que sur sa table", ("ni sur la table",))

    section("3. Chaque prise finit déposée, refusée ou reposée, avec une raison claire")
    jamais(res, "d'issue inattendue ni de refus sans raison", ("issue", "sans raison"))
    floues = [f"n° {x['carton']['numero']} : {x['raison']}" for x in res
              if x["issue"] != "déposée" and K.categorie(x["raison"]) == "autre"]
    check("Chaque raison de refus ou de repose est d'une catégorie connue (hors de portée, trop "
          "large, trop lourd, glisse…)", not floues, "; ".join(floues[:3]))
    exp = [x for x in res if x["carton"]["expres"]]
    check("Les cartons hors des moyens du robot ne sont jamais tenus pour marcher : refusés ou "
          "reposés", all(x["issue"] != "déposée" for x in exp),
          str([x["carton"]["numero"] for x in exp if x["issue"] == "déposée"]))
    # Pas de taux de réussite exigé ici : tirés dans toutes leurs plages à la
    # fois, peu de cartons tombent dans ce que le robot sait faire (rapport,
    # section 5) ; c'est une limite à montrer, pas à cacher. La prise réussie,
    # elle, est vérifiée au balayage autour du carton T1 (section 4).

    section("4. Un paramètre à la fois autour du carton T1")
    for (cle, v, _), x in zip(balayes, bal):
        print(f"    {cle:10s} {v:6.2f} : {x['issue']:8s} {x['raison'][:110]}")
    jamais(bal, "de carton tombé, écrasé, de plafond de couple dépassé ni de prise à vide",
           ("tombé", "écrasé", "plafond de couple", "limite matérielle", "mains vides",
            "sans que l'OS le voie", "ni sur la table", "issue", "sans raison"))
    check("Le carton T1 lui-même : pris, porté, déposé", bal[0]["issue"] == "déposée",
          f"{bal[0]['issue']} {bal[0]['raison']}")
    passes = {cle for (cle, _, _), x in zip(balayes[1:], bal[1:]) if x["issue"] == "déposée"}
    check(f"Autour de lui, chaque paramètre a des valeurs prises et déposées ({len(passes)} sur "
          f"{len(K.BALAYAGE)})", passes == {cle for cle, _ in K.BALAYAGE},
          str(sorted({cle for cle, _ in K.BALAYAGE} - passes)))

    section("4 bis. La repose, sur chaque carton du balayage tenu")
    tenus = [x for x in reposes if x["tenu"]]
    jamais(reposes, "de carton tombé en repose, ni laissé ailleurs que sur sa table, ni écrasé, "
           "ni de plafond de couple dépassé", ("tombé", "ni sur la table", "écrasé",
                                               "plafond de couple", "limite matérielle",
                                               "mains vides", "sans que l'OS le voie", "issue",
                                               "sans raison"))
    check(f"{len(tenus)} cartons tenus (autant qu'à la dépose : "
          f"{sum(x['tenu'] for x in bal)}), chacun reposé à sa place sur la table, bras libres, "
          f"jamais par l'interpolation des angles (POSE)",
          len(tenus) == sum(x["tenu"] for x in bal) and len(tenus) >= len(K.BALAYAGE)
          and all(x["issue"] != "bloquée" and x["sur_table"] and "POSE" not in x["etats"]
                  for x in tenus),
          str([(x["carton"]["numero"], x["issue"], x["raison"][:60]) for x in tenus
               if x["issue"] == "bloquée" or not x["sur_table"]][:3]))

    section("5. Rapport : les vraies limites du robot")
    print(K.rapport(res, GRAINE))

    section("6. Réglages choisis sur le carton T1, confrontés à la campagne")
    for ligne in K.audit(res + bal):
        print(f"  - {ligne}")
    print(f"\n  ({N + len(bal)} essais en {time.perf_counter() - t0:.0f} s)")

    print(f"\n{'=' * 66}")
    print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
    print("=" * 66)
    sys.exit(0 if FAIL == 0 else 1)
