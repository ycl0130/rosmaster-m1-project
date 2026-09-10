# Local Fast2D SCOPE causality test protocol

This test is test-only. It never sends a navigation action or publishes a velocity command.

## Fixed definitions

The frozen pose is sampled once at `TEST_READY`; the planning velocity is zero. The reference
is a 2.0 m straight path along positive local-frame Y, sampled at 0.05 m. The goal is its end.

The reference corridor half-width is **0.42 m**: the 0.15 m lateral footprint half extent,
0.02 m footprint padding, and 0.25 m of the production 0.40 m inflation region. This is a
pre-registered geometric safety corridor, not a result-derived threshold.

`T_scope` is the first of three consecutive SCOPE prediction frames with a scope-layer-risk
cell in the corridor. `T_scan` is the first of three consecutive scan frames having an observed
return within the corridor expanded by the 0.40 m production inflation radius. `T_local` is the
first of three consecutive guide updates whose maximum lateral deviation from the reference
exceeds 0.20 m. The analyzer also records integrated lateral deviation.

The sim-time epoch is set after costmap readiness (and SCOPE readiness for the ON trial). The
single obstacle begins at epoch + 4.0 s and is driven solely by simulation time at 0.15 m/s.
The trial contains 4 s pre-event and 4 s post-event windows and exits automatically.

## Implementation notes (Phase 3A)

The test harness obtains `/local_costmap/get_costmap` (`nav2_msgs/srv/GetCostmap`) every
planning cycle. This is Costmap2DPublisher's final layered master output, rather than an
individual layer; it therefore contains obstacle, SCOPE (when enabled), and inflation costs.
The harness and HybridController both call `planLocalFast2D`, which accepts this complete
snapshot and has no controller, lifecycle, costmap mutation, or publishing side effects.

The test-only mover receives the harness T0 state message, then starts at T0 + 4 s based on
`/clock`; its production `/cmd_vel` trigger remains the default. The recorder starts on the
same T0 message and terminates the rosbag on `TEST_COMPLETE`.
