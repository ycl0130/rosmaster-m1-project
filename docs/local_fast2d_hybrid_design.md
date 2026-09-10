# Local Fast2D / MPPI hybrid: architecture decision

## Decision

Use a `nav2_core::Controller` wrapper (`m1_local_fast2d::HybridController`) around
the stock `nav2_mppi_controller::MPPIController` (**option B**).  The wrapper receives
the global plan through Nav2's normal `Controller::setPlan()`, has the exact
`Costmap2DROS` instance owned by `controller_server`, and forwards either the original
plan or its newest valid local guide to MPPI.  The BT, planner server, Fast2D global
planner, costmaps, Collision Monitor, and watchdog are not changed.

`local_fast2d_mode:=off` is the default and launch leaves `FollowPath` bound directly
to the existing MPPI plugin.  It is consequently the byte-for-byte existing Nav2
controller configuration/path flow.  `shadow` and `active` select the wrapper.  In
shadow it publishes plans but always gives MPPI the original global plan.  In active it
uses a fresh, valid local guide, otherwise immediately forwards the original global
plan.

## Observed Nav2 data flow

`ComputePathToPose` stores a `nav_msgs/Path` in BT blackboard key `{path}` and
`FollowPath` submits it to `controller_server`.  `controller_server` calls the chosen
controller plugin's `setPlan()`, then calls `computeVelocityCommands()` at the
configured 20 Hz.  MPPI receives that plan via its own `setPlan()` and continues to
perform its normal costmap critics, footprint checking, omnidirectional `vx/vy/wz`
sampling, and final command computation.

`Controller::configure()` is passed the controller server's `local_costmap`
`Costmap2DROS`.  Its `getCostmap()` is the composed master grid (obstacle, SCOPE, then
inflation), so no Gazebo ground truth input is needed or used.

## Alternatives considered

* **A, separate node and adapter:** a normal topic node cannot replace the private
  action/controller `setPlan()` handoff without an additional controller or BT bridge;
  it also sees a serialized costmap topic rather than the controller's master grid.
* **B, controller wrapper (chosen):** one Nav2 plugin boundary; direct local-master
  costmap access; official MPPI remains intact; immediate in-process fallback; a worker
  never blocks the 20 Hz compute call.
* **C, BT insertion:** requires a custom BT action plus a new plan handoff and changes
  the validated navigation tree/recovery timing.  It is a larger, less reliable
  integration point for a receding-horizon component.

## MVP algorithm and safety contract

The worker snapshots the most recent global path, robot pose and velocity, transforms
the path into the local-costmap frame, chooses a monotonic 2.0 m lookahead (or final
goal inside the window), searches only inside the 5 x 5 m master grid, and invokes the
existing `(px, py, vx, vy)` constant-acceleration A* core.  Lethal/inscribed cells and
disallowed unknown cells reject a primitive.  non-lethal master costs—including SCOPE
medium risk and inflation—are sampled continuously as a soft edge penalty.  No time
state or ground-truth obstacle input is introduced.

A guide must begin near the current robot pose and pass freshness/validity checks.
Timeout, exception, no path, stale guide, and continuity rejection increment
diagnostics and select the original global path.  MPPI, Collision Monitor,
PolygonStop, and watchdog remain the final safety layers.

## Validation plan

First validate shadow at 5--10 Hz in straight, static, crossing, and SCOPE-on crossing
scenarios.  Only after that, validate active against the same scenario while recording
guide latency/replans/fallbacks, MPPI recovery signals, PolygonStop events, clearance,
and NavigateToPose result.  This MVP does not claim time-aware trajectory matching;
that is a later decision contingent on measured benefit.
