#!/usr/bin/env bash
set -euo pipefail

base_url="${1:-http://127.0.0.1:8000}"
request_id="deploy-v7-$(date +%s)"
temp_dir="$(mktemp -d /tmp/carpoint-worker-scan.XXXXXX)"
trap 'rm -rf -- "$temp_dir"' EXIT

post() {
  local name="$1"
  local url="$2"
  local code
  code="$(curl -sS -o "$temp_dir/$name.json" -w '%{http_code}' -X POST -d '' "$url")"
  printf '%s HTTP=%s ' "$name" "$code"
  cat "$temp_dir/$name.json"
  printf '\n'
  printf '%s' "$code"
}

printf 'requestId=%s\n' "$request_id"
first_code="$(post first "$base_url/api/radars/scan-once?requestId=$request_id" | tee /dev/stderr | tail -c 3)"
[[ "$first_code" == "202" ]]

duplicate_code="$(post duplicate "$base_url/api/radars/scan-once?requestId=$request_id" | tee /dev/stderr | tail -c 3)"
[[ "$duplicate_code" == "200" ]]

status="$(curl -fsS "$base_url/api/radars/scan-once")"
printf 'activeStatus=%s\n' "$status"
grep -q "\"requestId\":\"$request_id\"" <<< "$status"

stop_code="$(post stop "$base_url/api/radars/scan-once/stop" | tee /dev/stderr | tail -c 3)"
[[ "$stop_code" == "200" || "$stop_code" == "202" ]]

ready=false
for _ in {1..20}; do
  status="$(curl -fsS "$base_url/api/radars/scan-once")"
  if grep -q '"state":"ready"' <<< "$status"; then
    ready=true
    break
  fi
  sleep 1
done
printf 'finalStatus=%s\n' "$status"
[[ "$ready" == "true" ]]
printf '{"passed":true,"requestId":"%s","first":202,"duplicate":200,"stoppedAndReady":true}\n' "$request_id"
