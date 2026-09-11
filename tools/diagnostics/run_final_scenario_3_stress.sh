#!/usr/bin/env bash
# Build then run the single-session Scenario 3 final stress validation.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="${FINAL_SCENARIO_3_OUTPUT:-$ROOT/artifacts/final_scenario_3_$(date +%Y%m%d_%H%M%S)}"
MODEL="${M1_SCOPE_MODEL_PATH:-/home/lin24311/Data/SCOPE-repro/reference/scope/model/scope_model.pth}"
if [[ ! -f "$MODEL" ]]; then
  echo "SCOPE model missing: $MODEL" >&2
  exit 2
fi
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
cd "$ROOT"
colcon build --packages-select m1_local_fast2d m1_nav2_bringup m1_scope_predictor m1_scope_msgs --symlink-install
source install/setup.bash
export M1_SCOPE_PYTHON="${M1_SCOPE_PYTHON:-/home/lin24311/car_ws/.venv-ros2-ml/bin/python}"
if [[ ! -x "$M1_SCOPE_PYTHON" ]]; then
  echo "SCOPE Python missing or not executable: $M1_SCOPE_PYTHON" >&2
  exit 2
fi
exec python3 tools/diagnostics/final_scenario_3_stress.py \
  --duration "${FINAL_SCENARIO_3_DURATION:-1800}" --scope-model-path "$MODEL" --output "$OUT"
