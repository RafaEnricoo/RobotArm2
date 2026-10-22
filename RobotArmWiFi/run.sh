#!/usr/bin/env bash
# Jalankan server robot arm. Venv & dependensi disiapkan otomatis kalau belum ada.
cd "$(dirname "$0")"
if [ ! -x .venv/bin/python ]; then
  echo "Menyiapkan venv..."
  python3 -m venv .venv && .venv/bin/pip install -q -r requirements.txt || exit 1
fi
exec .venv/bin/python server.py "$@"
