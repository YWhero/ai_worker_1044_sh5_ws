#!/bin/sh
set -eu

case "${SERVICE_NAME:-}" in
  ''|*[!A-Za-z0-9_-]*) exit 1 ;;
esac
PGID_FILE="${CYCLO_SERVICE_RUN_DIR:-/run}/${SERVICE_NAME}.pgid"
[ -f "$PGID_FILE" ] || exit 0
PGID="$(cat "$PGID_FILE")"
case "$PGID" in
  ''|*[!0-9]*) exit 1 ;;
esac
OWN_PGID="$(ps -o pgid= -p $$ | tr -d ' ')"
[ "$PGID" -gt 1 ] && [ "$PGID" != "$OWN_PGID" ] || exit 1

printf '[%s finish] Stopping process group %s\n' "$SERVICE_NAME" "$PGID"
kill -TERM -"$PGID" 2>/dev/null || true
ATTEMPT=0
while kill -0 -"$PGID" 2>/dev/null && [ "$ATTEMPT" -lt 20 ]; do
  sleep 0.1
  ATTEMPT=$((ATTEMPT + 1))
done
if kill -0 -"$PGID" 2>/dev/null; then
  printf '[%s finish] Forcing remaining process group %s to exit\n' "$SERVICE_NAME" "$PGID"
  kill -KILL -"$PGID" 2>/dev/null || true
fi
rm -f "$PGID_FILE"
printf '[%s finish] Cleanup completed\n' "$SERVICE_NAME"
