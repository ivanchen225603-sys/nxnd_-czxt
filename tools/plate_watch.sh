#!/bin/sh
set -eu
script=/data/carpointcloud/ops/plate-watch/plate_watch.py
if [ "${1:-}" = run ]; then
    app=$(systemctl show -p WorkingDirectory --value carpointcloudwebapi.service)
    runtime=$(dirname "$(dirname "$app")")/runtime/venv/bin/python
    exec "$runtime" -B -u "$script" "$@"
fi
exec /usr/bin/python3 -B -u "$script" view "$@"
