# RestrictedTimedReference contract

`m1_fast_planner::PlanarState::vx` and `vy` are planning-frame/world-frame
velocities.  The proof is the planner's double-integrator propagation:

```text
px' = px + vx * dt + 0.5 * ax * dt^2
py' = py + vy * dt + 0.5 * ay * dt^2
vx' = vx + ax * dt
vy' = vy + ay * dt
```

`Fast2DPlanner` obtains the initial state by rotating odometry body velocity
into `global_frame_` with `rotateBodyVelocityToPlanningFrame()`.  Therefore the
selected `TimedTrajectory` keeps `vx/vy` in its planning frame (normally
`odom`); it is not a body-frame command and it is not a path-tangent frame.

For a body-frame reference, the deterministic conversion is:

```text
vx_body = cos(yaw) * vx_world + sin(yaw) * vy_world
vy_body = -sin(yaw) * vx_world + cos(yaw) * vy_world
```

Sampling uses the controller's activation-relative elapsed time.  Between
strictly positive-time samples, x/y, vx/vy, and ax/ay are linearly
interpolated.  Yaw is linearly interpolated only after shortest-angle unwrap;
the interval omega is the bounded, piecewise-constant value computed from that
unwrapped yaw difference.  A query at or beyond the terminal time returns the
terminal pose with zero reference velocity, acceleration, and omega.

The yaw remains the selected trajectory's geometric heading reference.  It is
not asserted to be the optimal Mecanum chassis orientation; tangent-yaw versus
relaxed body-yaw tracking remains a later-phase question.

`RestrictedTimedReference` uses a new activation timestamp as tracking time
zero.  The original trajectory header stamp remains the planning snapshot
stamp for diagnostics only.  It unwraps yaw before interpolation, computes a
finite bounded omega reference from unwrapped yaw differences, excludes
zero-duration terminal connectors from dynamic intervals, and holds the
terminal pose with zero reference velocity after the trajectory ends.

Phase 6B feeds the immutable horizon into the stock Humble 1.1.20 optimizer
through the project-local `RestrictedMPPIAdapter`.  The adapter expands only
the official `generateNoisedTrajectories()` seam: each sampled control is
formed as `u_ref + delta_u`, intersected with the absolute MPPI envelope, and
hard-clamped before propagation.  After the unchanged official critics run,
the adapter writes `+infinity` for every rollout outside the time-indexed
position tube or in collision.  It handles the all-infeasible case before the
official softmax so it cannot turn into NaN or unrestricted fallback.

The adapter uses a narrow `protected` access shim because 1.1.20 does not
publish a nominal-control or rollout-mask API.  The installed package version
is read at runtime and restricted mode is refused unless it is exactly
`1.1.20`.  Feature OFF continues through the official
`MPPIController::computeVelocityCommands()` entry point, with no change to
its parameters, sampling, critics, or command behavior.  No file under
`/opt/ros/humble` is modified and the upstream controller is not copied into
the workspace.
