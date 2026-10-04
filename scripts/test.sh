#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
python -m unittest discover -s tests -v
(cd dashboard/frontend && npm ci && npm run build)
if command -v colcon >/dev/null; then
  (cd drone_ws && colcon test && colcon test-result --verbose)
fi
