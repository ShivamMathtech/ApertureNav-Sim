#!/usr/bin/env bash
set -eo pipefail
curl --fail-with-body -X POST http://127.0.0.1:8000/api/command -H 'Content-Type: application/json' -d '{"action":"reset"}'
