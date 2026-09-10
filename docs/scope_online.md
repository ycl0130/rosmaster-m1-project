# M1 SCOPE online observer and optional local-costmap layer

By default, this package runs the frozen TempleRAIL SCOPE model beside Nav2 as
an observer. It reads `/scan` and `/odom` and publishes prediction products,
but does not publish velocity commands or affect MPPI. The Gazebo Nav2 launch
also has an optional local-costmap feedback entry. That entry is a closed-loop
use of the prediction because MPPI can see the resulting local costs, but it is
disabled by default with `scope_enabled:=false`.

## Runtime environment

The strict `scope-repro` environment remains the reference baseline. Build the
ROS 2 runtime in a separate Python 3.10 virtual environment:

```bash
python3 -m venv --system-site-packages .venv-scope-runtime
.venv-scope-runtime/bin/python -m pip install \
  -r src/m1_scope_predictor/requirements-cu124.txt
source /opt/ros/humble/setup.bash
source .venv-scope-runtime/bin/activate
/usr/bin/colcon build --packages-up-to \
  m1_scope_bridge m1_scope_predictor m1_scope_costmap_layer \
  m1_nav2_bringup --symlink-install
source install/setup.bash
```

The package contains byte-for-byte copies of the two MIT-licensed model source
files from commit `211b199ebfcbe54eeac99202bbe62aca593f7bed`. The checkpoint
is external, hash-checked at startup, and is not committed to this repository.

The costmap layer does not alter the official model or grid contract. SCOPE
still consumes ten 10 Hz frames as `[N,10,1,64,64]`; each grid is 64 x 64 at
0.1 m/cell with model axes x in `[0,6.4)` m and y in `[-3.2,3.2)` m. The
default online prediction is two autoregressive steps (0.2 s); the parallel
typed sequence can be configured from two to twenty steps without changing the
legacy endpoint topics.

## Standalone use

Start the existing Gazebo/Nav2 launch first, without its RViz instance:

```bash
ros2 launch m1_nav2_bringup nav2_m1_gazebo.launch.py gui:=true rviz:=false
```

Then start the observer and its dedicated RViz configuration:

```bash
ros2 launch m1_scope_predictor scope_online.launch.py \
  use_sim_time:=true rviz:=true \
  model_path:=/actual/path/to/scope_model.pth
```

The first prediction appears after the ten-frame history warms up. The default
uses a 0.2 s horizon and four Monte Carlo samples. Full SCOPE prioritizes a
fresh latest result over processing every nominal 10 Hz job.

To enable the Gazebo-only delayed evaluator, add
`evaluator_enabled:=true`. It uses `/ground_truth/odom` only to align the future
LaserScan with the prediction frame and reports whole endpoint-grid IoU, F1,
and MAE through `/scope/diagnostics`.

## Topics and frames

- `/scope/current_ogm`: current scan endpoints. `100` is occupied and `-1`
  means not marked by a scan endpoint; it must not be interpreted as free.
- `/scope/prediction`: dense future occupancy probability mapped to `0..100`.
- `/scope/uncertainty`: pixel standard deviation divided by `0.5`, clipped,
  and mapped to `0..100` for transport and display.
- `/scope/prediction_sequence`: atomic typed `ScopePredictionSequence` with
  one probability/uncertainty pair per 0.1 s autoregressive step. Its header
  is the observation anchor; each slice grid is stamped anchor plus its
  `time_from_start`. `sequence_horizon_steps` defaults to 2 and may be 2..20.
  `/scope/prediction` and `/scope/uncertainty` always remain step 2 (t+0.2 s),
  even if this parallel sequence is extended.
- `/scope/diagnostics`: history state, inference latency, result age, output
  rate, dropped jobs, allocated CUDA memory, and optional evaluator metrics.

`/scope/current_ogm` and `/scope/diagnostics` remain observer-facing products.
When the optional local layer is enabled, `/scope/prediction` and
`/scope/uncertainty` are its exact matched input pair; when it is disabled,
those two grids remain non-contributing prediction/visualization outputs.

All grids use the `odom` frame. The current grid origin is the current laser
pose composed with `(0,-3.2)` m; prediction and uncertainty use the predicted
future laser pose. Internally, arrays remain float tensors and are not
round-tripped through `OccupancyGrid`.

## Optional local-costmap closed loop

After standalone validation, the observer and local layer can be enabled
together:

```bash
ros2 launch m1_nav2_bringup nav2_m1_gazebo.launch.py \
  scope_enabled:=true scope_num_samples:=4
```

`scope_enabled` defaults to `false`. These outputs are visualization and
measurement artifacts while it is false. In the combined launch, the same
argument jointly gates the SCOPE predictor launch and rewrites only
`local_costmap.local_costmap.ros__parameters.scope_layer.enabled`. The local
plugin order is exactly:

```yaml
plugins: [obstacle_layer, scope_layer, inflation_layer]
```

Thus measured obstacles are established first, SCOPE can only add predicted
cost, and inflation runs last. The global costmap plugin list and behavior are
unchanged; SCOPE is local-only.

The layer accepts a prediction/uncertainty pair only when the two
`OccupancyGrid` messages have exactly the same timestamp, frame, width, height,
resolution, and origin pose. Their frame must also equal the local costmap's
global frame (`odom` in this launch). Each source cell is projected using the
grid origin's planar rotation. Rasterization uses positive-area polygon/cell
intersection and clips candidates to both the master map and the current
costmap update window, including rotated grids and unequal resolutions.

For known encoded values `p_enc` and `u_enc`, the layer decodes and fuses risk
as

```text
p     = p_enc / 100
sigma = uncertainty_encoding_scale * (u_enc / 100)
risk  = min(1, p + uncertainty_gain * sigma)
```

The predictor encodes standard deviation by dividing it by 0.5 before mapping
it to `0..100`, so the defaults `uncertainty_encoding_scale=0.5` and
`uncertainty_gain=1.0` recover `risk=min(1,p+sigma)`. A `-1` in either input is
unknown and transparent. With the default thresholds, risk below 0.35 is
transparent, `[0.35,0.60)` proposes medium cost 200, and risk at least 0.60
proposes Nav2 lethal cost 254.

Merge behavior is transparent and maximum-only: the layer never clears or
reduces an existing master cost, and it writes a SCOPE cost only when that cost
raises the master cell (or the master cell is `NO_INFORMATION`). A valid render
emits a throttled INFO diagnostic at most once per second in this stable form:

```text
SCOPE raster stats valid=<n> clipped=<n> medium=<n> lethal=<n>
```

`valid` counts known prediction/uncertainty source cells; `clipped` counts
valid source cells not fully contained by the active update window; `medium`
and `lethal` count master cells actually raised by SCOPE in that render. These
fields can therefore be captured directly from ordinary launch logs without an
extra topic.

Freshness is based on local receipt time. When the matched pair is older than
0.5 s, the layer requests recomposition of its old bounds and withdraws only
its own contribution. It deliberately remains current, so stale or absent
SCOPE output never makes the local costmap or controller stale. A fresh matched
pair can contribute again on a later update.

This integration is optional and default-off. Seeing the predictor, layer, and
controller run together proves wiring and permits closed-loop experiments; it
does not by itself demonstrate navigation-performance improvement, real-time
guarantees, or real-hardware safety.
