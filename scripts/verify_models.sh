#!/bin/bash
# Check the model artefacts in a checkout against scripts/models.manifest.
# Run before launching a campaign block on any host. Exit 1 on any mismatch.
#   scripts/verify_models.sh [repo_root]
set -u
ROOT="${1:-$(git rev-parse --show-toplevel)}"
MAN="$(dirname "$0")/models.manifest"
bad=0
while read -r h sz p; do
  case "$h" in \#*|'') continue;; esac
  f="$ROOT/$p"
  if [ ! -f "$f" ]; then echo "MISSING   $p"; bad=1; continue; fi
  as=$(stat -Lc%s "$f"); ah=$(md5sum "$f" | cut -c1-16)
  if [ "$as" != "$sz" ]; then echo "SIZE      $p (expected $sz, got $as)"; bad=1
  elif [ "$ah" != "$h" ]; then echo "CONTENT   $p (expected $h, got $ah)"; bad=1
  else echo "ok        $p"; fi
done < "$MAN"
[ $bad -eq 0 ] && echo "all model artefacts match the manifest" || echo "MODEL MISMATCH: results from this host are not comparable"
exit $bad
