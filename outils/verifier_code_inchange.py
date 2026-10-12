#!/usr/bin/env python3
"""
Preuve qu'un changement ne touche que les commentaires et les docstrings.

Pourquoi : réécrire des commentaires dans du code de sécurité est utile
seulement si le code, lui, reste exactement le même. Relire un diff ne
suffit pas à le garantir (une indentation, un caractère déplacé passent
facilement inaperçus). Ce script le vérifie mécaniquement, fichier par
fichier, entre deux révisions git.

Ce qui est comparé, selon le type de fichier :

  .py       l'arbre syntaxique (AST) des deux versions, après retrait des
            docstrings (la chaîne placée en première instruction d'un
            module, d'une classe ou d'une fonction) et de toute autre
            instruction qui n'est qu'une chaîne littérale, comme les
            « docstrings d'attribut » : Python les évalue puis les jette,
            sans effet. Les commentaires « # »
            n'existent pas dans l'AST : ils sont ignorés d'office. Tout le
            reste (noms, valeurs, ordre des instructions) doit être
            identique.
  .sh       les lignes, sans les lignes de commentaire ni les lignes vides.
            La première ligne « #! » compte comme du code. Le contenu d'un
            heredoc (« <<FIN ... FIN ») est gardé tel quel, commentaires
            compris : c'est une donnée, pas un commentaire du script. Un
            commentaire en fin de ligne de code est comparé avec la ligne :
            le modifier compte comme une différence. Limite : une ligne qui
            commence par « # » à l'intérieur d'une chaîne entre guillemets
            écrite sur plusieurs lignes serait prise pour un commentaire.
            Aucun script du dépôt n'en contient (vérifié le 2026-10-11) ;
            ne pas modifier une telle ligne sans la relire à la main.
  .c, .h    le code après retrait des commentaires par le préprocesseur C
            (gcc -fpreprocessed).

Un fichier renommé est comparé à son ancienne version. Un fichier ajouté ou
supprimé est signalé : il n'y a rien à comparer. Les autres fichiers
(Markdown, JSON, texte…) sont listés sans être vérifiés.

Usage :
    python3 outils/verifier_code_inchange.py              # HEAD~1 -> HEAD
    python3 outils/verifier_code_inchange.py AVANT APRES  # deux révisions
    python3 outils/verifier_code_inchange.py AVANT        # AVANT -> fichiers non committés

Code de sortie : 0 si aucune différence de code, 1 sinon, 2 si la
vérification n'a pas pu se faire.
"""
import ast
import re
import shutil
import subprocess
import sys
from pathlib import Path

RACINE = Path(__file__).resolve().parents[1]

DEBUT_HEREDOC = re.compile(r"<<(-?)\s*(['\"]?)([A-Za-z_][A-Za-z0-9_]*)\2")


def git(*args, binaire=False):
    r = subprocess.run(["git", "-C", str(RACINE), *args], capture_output=True)
    if r.returncode != 0:
        raise RuntimeError(f"git {' '.join(args)} : {r.stderr.decode(errors='replace').strip()}")
    return r.stdout if binaire else r.stdout.decode("utf-8")


def contenu(revision, chemin):
    """Contenu d'un fichier à une révision, ou dans l'arbre de travail si
    revision vaut None."""
    if revision is None:
        return (RACINE / chemin).read_text(encoding="utf-8")
    return git("show", f"{revision}:{chemin}", binaire=True).decode("utf-8")


# --- Python -----------------------------------------------------------------

def _est_chaine_seule(instruction):
    """Vrai pour une instruction qui n'est qu'une chaîne littérale : une
    docstring, ou une « docstring d'attribut » placée sous un attribut de
    classe. Python l'évalue puis la jette, sans aucun effet : la retirer ne
    change rien à ce que fait le programme. Une f-string, elle, peut appeler
    du code : elle n'est pas concernée."""
    return (isinstance(instruction, ast.Expr)
            and isinstance(instruction.value, ast.Constant)
            and isinstance(instruction.value.value, str))


def _sans_docstrings(arbre):
    for noeud in ast.walk(arbre):
        for champ in ("body", "orelse", "finalbody"):
            corps = getattr(noeud, champ, None)
            if isinstance(corps, list):
                setattr(noeud, champ, [i for i in corps if not _est_chaine_seule(i)])
    return arbre


def code_python(texte):
    return ast.dump(_sans_docstrings(ast.parse(texte)), include_attributes=False)


# --- Shell ------------------------------------------------------------------

def code_shell(texte):
    garde, fin_heredoc = [], []
    for numero, ligne in enumerate(texte.splitlines()):
        if fin_heredoc:
            garde.append(ligne)
            delimiteur, tabulations = fin_heredoc[0]
            if (ligne.lstrip("\t") if tabulations else ligne) == delimiteur:
                fin_heredoc.pop(0)
            continue
        nette = ligne.strip()
        if numero == 0 and nette.startswith("#!"):
            garde.append(ligne)
            continue
        if not nette or nette.startswith("#"):
            continue
        garde.append(ligne)
        for m in DEBUT_HEREDOC.finditer(ligne):
            fin_heredoc.append((m.group(3), m.group(1) == "-"))
    return "\n".join(garde)


# --- C ----------------------------------------------------------------------

def code_c(texte):
    compilateur = shutil.which("gcc") or shutil.which("cc")
    if compilateur is None:
        raise RuntimeError("gcc introuvable : impossible de retirer les commentaires C")
    r = subprocess.run([compilateur, "-fpreprocessed", "-dD", "-E", "-P", "-x", "c", "-"],
                       input=texte, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"préprocesseur C : {r.stderr.strip()}")
    return "\n".join(l.rstrip() for l in r.stdout.splitlines() if l.strip())


EXTRACTEURS = {".py": code_python, ".sh": code_shell, ".c": code_c, ".h": code_c}


def fichiers_changes(avant, apres):
    """Liste de (statut, ancien chemin, nouveau chemin)."""
    args = ["diff", "--name-status", "-M", avant] + ([apres] if apres else [])
    changes = []
    for ligne in git(*args).splitlines():
        champs = ligne.split("\t")
        statut = champs[0][0]
        if statut == "R":
            changes.append(("R", champs[1], champs[2]))
        else:
            changes.append((statut, champs[1], champs[1]))
    if apres is None:
        # Fichiers nouveaux pas encore suivis : signalés comme ajoutés.
        for chemin in git("ls-files", "--others", "--exclude-standard").splitlines():
            changes.append(("A", chemin, chemin))
    return changes


def main(argv):
    avant = argv[1] if len(argv) > 1 else "HEAD~1"
    apres = argv[2] if len(argv) > 2 else (None if len(argv) == 2 else "HEAD")
    try:
        changes = fichiers_changes(avant, apres)
    except RuntimeError as e:
        print(f"ERREUR : {e}")
        return 2

    verifies, differences, signales, non_verifies = 0, [], [], []
    for statut, ancien, nouveau in changes:
        extraire = EXTRACTEURS.get(Path(nouveau).suffix)
        if statut in ("A", "D"):
            signales.append(f"{'ajouté' if statut == 'A' else 'supprimé'} : {nouveau}")
            continue
        if extraire is None or Path(ancien).suffix != Path(nouveau).suffix:
            non_verifies.append(nouveau)
            continue
        try:
            identique = extraire(contenu(avant, ancien)) == extraire(contenu(apres, nouveau))
        except (RuntimeError, SyntaxError, UnicodeDecodeError) as e:
            print(f"ERREUR sur {nouveau} : {e}")
            return 2
        verifies += 1
        if not identique:
            differences.append(nouveau if ancien == nouveau else f"{ancien} -> {nouveau}")

    cible = apres or "arbre de travail"
    print(f"Comparaison {avant} -> {cible}")
    print(f"  {verifies} fichier(s) de code comparé(s) (.py, .sh, .c, .h)")
    for ligne in signales:
        print(f"  {ligne} (rien à comparer)")
    if non_verifies:
        print(f"  {len(non_verifies)} autre(s) fichier(s) non vérifié(s) : {', '.join(non_verifies)}")
    if differences:
        print(f"  DIFFÉRENCE DE CODE dans {len(differences)} fichier(s) :")
        for chemin in differences:
            print(f"    {chemin}")
        return 1
    print("  Aucune différence de code : seuls des commentaires ou des docstrings ont changé.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
