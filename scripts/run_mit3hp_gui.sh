#!/bin/bash
set -e
cd "$(dirname "$0")/../build-sim"
exec ./sim/mit3hp_gui
