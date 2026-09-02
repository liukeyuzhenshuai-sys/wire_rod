#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
python run_web.py --host 127.0.0.1 --port 9999
