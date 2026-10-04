#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
python -m aperturenav.cli "$@"
