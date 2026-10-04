#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
if [[ ! -f dashboard/frontend/dist/index.html ]]; then
  (cd dashboard/frontend && npm ci && npm run build)
fi
exec python -m uvicorn dashboard.backend.app:app --host 127.0.0.1 --port 8000
