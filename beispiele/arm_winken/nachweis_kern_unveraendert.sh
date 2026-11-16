#!/bin/bash
# Nachweis: die Commits des Beispielprojekts aendern nichts ausserhalb von beispiele/.
# Aufruf aus dem Repo-Wurzelordner: bash beispiele/arm_winken/nachweis_kern_unveraendert.sh
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"
DIR=beispiele/arm_winken
COMMITS=$(git log --format=%H -- "$DIR")
if [ -z "$COMMITS" ]; then
  echo "Noch kein Commit fuer $DIR (nur Arbeitskopie):"
  echo "git status -- . ':!$DIR'"; git status --short -- . ":!$DIR"
  exit 0
fi
FAIL=0
for c in $COMMITS; do
  echo "== Commit $(git log -1 --format='%h %ad %s' --date=short "$c")"
  echo "-- alle geaenderten Dateien:"
  git diff --stat "$c^" "$c"
  OUT=$(git diff --name-only "$c^" "$c" -- . ":!$DIR")
  if [ -n "$OUT" ]; then
    echo "!! ausserhalb von $DIR geaendert:"; echo "$OUT"; FAIL=1
  else
    echo "-- git diff $c^ $c -- . ':!$DIR'  ->  leer (Kern unveraendert)"
  fi
done
exit $FAIL
