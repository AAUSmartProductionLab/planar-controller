#!/bin/bash
set -e

echo "=== Planar controller ==="
# The table layout is read from the PMC when the PlanarTable module resets.
# Arguments are passed on: see python -m planar --help.
exec python -m planar "$@"
