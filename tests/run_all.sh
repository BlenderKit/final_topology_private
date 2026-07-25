#!/bin/bash
# Run every test suite headlessly in background Blender.
#
#   ./run_all.sh                all suites
#   ./run_all.sh test_undo.py   one suite (any glob works)
#   BLENDER=/path/to/blender ./run_all.sh
#
# Every suite prints PASS/FAIL lines and ends with ALL PASSED or FAILURES.
# The installed addon must be the PRO variant (extras.py present), since the
# constraints live there.

BLENDER="${BLENDER:-/Applications/Blender5.0.app/Contents/MacOS/Blender}"
DIR="$(cd "$(dirname "$0")" && pwd)"
pattern="${1:-test_*.py}"

failed=0
total=0
for t in "$DIR"/$pattern; do
    [ -f "$t" ] || continue
    name=$(basename "$t")
    total=$((total + 1))
    out=$("$BLENDER" --background --python "$t" 2>&1)
    summary=$(echo "$out" | grep -E "ALL PASSED|FAILURES" | tail -1)
    if [ "$summary" = "ALL PASSED" ]; then
        echo "PASS  $name"
    else
        echo "FAIL  $name: ${summary:-no summary - crashed?}"
        echo "$out" | grep -E "^FAIL |Traceback|^[A-Za-z]*Error" | head -10 | sed 's/^/      /'
        failed=$((failed + 1))
    fi
done

echo "----"
echo "$((total - failed))/$total suites passed"
[ "$failed" -eq 0 ]
