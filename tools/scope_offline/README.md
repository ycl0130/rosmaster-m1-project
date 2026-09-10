# Offline pretrained SCOPE evaluation

This directory intentionally stays outside the ROS 2 package. The bridge exports and preprocesses
data with the ROS workspace environment; pretrained inference uses the frozen `scope-repro` Conda
environment (Python 3.7, PyTorch 1.7.1). The official SCOPE checkout is imported without edits.

## Inputs and preprocessing

After building and sourcing `m1_scope_bridge`, export a recorded bag and build 0.5 s-horizon OGMs:

```bash
ros2 run m1_scope_bridge bag_export \
  --bag artifacts/scope_m1/bags/scope_m1_baseline \
  --output-dir artifacts/scope_m1/raw

ros2 run m1_scope_bridge scope_preprocess \
  --dataset artifacts/scope_m1/raw/raw_dataset.npz \
  --output artifacts/scope_m1/preprocessed/scope_h05.npz \
  --horizon 5 --roi-radius 0.45
```

The tensor contract is `[N,10,1,64,64]` input and `[N,1,64,64]` target at 10 Hz. The grid is
0.1 m/cell over `x=[0,6.4)` m forward and `y=[-3.2,3.2)` m left. Input history is ego-motion
compensated into the predicted future lidar frame; the target is the directly observed future scan.

## Evaluation

```bash
conda run -n scope-repro /usr/bin/env PYTHONDONTWRITEBYTECODE=1 \
  python tools/scope_offline/evaluate_pretrained.py \
  --dataset artifacts/scope_m1/preprocessed/scope_h05.npz \
  --scope-root /actual/path/to/SCOPE-repro/reference/scope \
  --checkpoint /actual/path/to/scope_model.pth \
  --horizon 5 --num-samples 32 --max-windows 200 --min-windows 100 \
  --threshold 0.5 --seed 1337 --device auto \
  --output-dir artifacts/scope_m1/evaluation/h05
```

The evaluator selects evenly spaced windows deterministically, reports micro and macro occupied-cell
IoU/F1/precision/recall plus MAE, and evaluates both the full grid and a 0.45 m dynamic-obstacle ROI.
Undefined per-window ratios are excluded from macro means and counted. `copy_last_ogm` is the
persistence baseline. Latency covers five autoregressive forwards for all samples in one window.

Generate the deterministic dynamic-obstacle diagnostic. It selects the evaluated window with the
largest count of observed future occupied cells inside its dynamic ROI; it does not use prediction
metrics for selection:

```bash
conda run -n scope-repro /usr/bin/env MPLBACKEND=Agg MPLCONFIGDIR=/tmp/scope-m1-mpl \
  PYTHONDONTWRITEBYTECODE=1 python tools/scope_offline/visualize_prediction.py \
  --dataset artifacts/scope_m1/preprocessed/scope_h05.npz \
  --predictions artifacts/scope_m1/evaluation/h05/predictions_h05.npz \
  --output artifacts/scope_m1/evaluation/h05/five_panel.png
```

For a matched timeline containing `t-0.9`, `t-0.6`, `t-0.3`, and `t`, plus SCOPE prediction and
observed target at both `t+0.5` and `t+1.0`, use `visualize_timeline.py`. Its two prediction inputs
must contain the same anchor timestamp; `--h05-dataset-index` selects that shared scene.

Large bags, arrays, and predictions live below ignored `artifacts/scope_m1/`. Only compact summaries
and one diagnostic image belong in Git.
