# franka_streaming_impedance_controller

A streaming Cartesian impedance controller for the Franka FR3, driven by **absolutely-timed pose
chunks**. It splices consecutive chunks on their own waypoint times, so the arm neither starts from
rest nor stops at a chunk boundary — a chunked reference (a policy's action chunks, a planner's
sampled path) keeps its timeline all the way down to joint torques. Built for contact-rich work: a
real Cartesian mass-spring-damper with a bounded commanded force, about any TF frame you name.

Needs `franka_ros2`, `libfranka`, and a realtime kernel — this is a 1 kHz torque controller.
Developed against ROS 2 Humble; the Python client is distro-agnostic.

| Package | Build type | What it is |
|---|---|---|
| `franka_streaming_impedance_controller` | `ament_cmake` | The ros2_control plugin, the law and interpolators as a separately testable library, and `franka_hand_node` |
| `franka_streaming_impedance_client` | `ament_python` | Producer side: builds the timed `MultiDOFJointTrajectory` chunks the controller consumes |

## Build and run

```bash
# into a workspace that already has franka_ros2 + libfranka
REPO=/path/to/franka_streaming_impedance_controller
ln -s $REPO/franka_streaming_impedance_controller $REPO/franka_streaming_impedance_client src/
colcon build --packages-select franka_streaming_impedance_controller franka_streaming_impedance_client \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select franka_streaming_impedance_controller franka_streaming_impedance_client

ros2 run controller_manager spawner cartesian_impedance_controller \
  -t franka_streaming_impedance_controller/CartesianImpedanceController \
  --param-file config/controllers.example.yaml --inactive
```

Tests need no hardware — the law and interpolators build as `franka_streaming_impedance_core`,
which has no hardware interface. Spawn **inactive**, then `switch_controller` to it: it claims the
same `<joint>/effort` interfaces as your joint-trajectory controller, so exactly one of the two can
hold the arm, and switching restarts the libfranka control loop — **only switch with the arm
stationary**. Every parameter and the reasoning behind the shipped gains lives in
[config/controllers.example.yaml](config/controllers.example.yaml).

## The wire contract

`target_topic` takes `trajectory_msgs/MultiDOFJointTrajectory`. Three rules; the first two make
the controller drop a chunk rather than guess at it:

- **`header.frame_id` == the controller's `base_frame`**, or empty to mean "base frame assumed".
  Poses are commanded in the base frame, and a chunk naming any *other* frame is dropped.
- **`joint_names` == exactly `[tcp_frame]`.** It names the *commanded frame*, not a joint, so a
  chunk aimed elsewhere is not reinterpreted just because it landed on this topic.
- **Waypoint times are absolute** (`header.stamp + time_from_start`). This is what makes splicing
  work: a new chunk overwrites the old one only from its own first waypoint onward, and whatever
  is still pending before that keeps playing. Stamping a chunk in the past is therefore how you
  compensate action latency — its early waypoints are simply already due.

```python
from franka_streaming_impedance_client.target_chunk import TargetChunkPublisher

pub = TargetChunkPublisher(
    node,
    frame_id='fr3_link0',            # must match base_frame
    joint_name='fr3_hand_tcp',       # must match tcp_frame
    topic='/cartesian_impedance_controller/target_poses',
)
pub.publish(poses, dt=0.1)           # poses land at now + k*dt
```

`topic` is deliberately required: the controller's default is node-relative, so a client-side
default would resolve against the *publishing* node and address nothing.
`get_subscription_count()` is exposed because publishing where nothing subscribes is otherwise
completely silent. If you slice stale waypoints off the front of a chunk, keep numbering from the
*unsliced* index (`first_index`) — renumbering the survivors from zero shifts the whole timeline
earlier, which looks like tracking lag rather than a bug.

## Three design notes

**A custom `tcp_frame`** matters because a spring anchored somewhere other than the real contact
point turns an orientation error into a large translation where it counts. Pose and Jacobian are
both moved onto it at activation with one TF lookup. (`SetTCPFrame` is not the lever — it changes
`O_T_EE` reporting only, while TF and MoveIt are driven by the URDF.)

**Torque rather than `cartesian_pose`** because under that interface libfranka hardcodes
`ControllerMode::kJointImpedance` — stiffness-only, no damping knob — and hardcodes its rate
limiter and low-pass filter off, so nothing downstream catches a discontinuity either.

**Position is a cubic Hermite spline, not lerp**, which is what makes "neither starts from rest nor
stops at a chunk boundary" true of *velocity*. Each waypoint gets one shared slope between its incoming and outgoing segments, so the reference's velocity is continuous
through interior waypoints and across splices. Tangents are clamped monotone, so the spline cannot
locally outrun the average speed `max_pos_speed` already bounds — and that distance is what sets
contact force. Orientation is still slerp, so its angular velocity still steps at each waypoint. Position is continous for both linear and orientation controllers. 

## `franka_hand_node`

An optional driver for a stock Franka Hand: `JointTrajectory` widths in metres on `target_topic`
(default `~/target_widths`), finger state on `~/joint_states`. It uses libfranka directly rather
than `franka_gripper` because a `Move` cannot be pre-empted — `stop()` queues *behind* it, and a
superseding `Move` stops the fingers rather than redirecting them. Owning the connection makes
run-to-completion structural: one thread calls `move()`, in a loop. It also holds the libfranka
gripper connection, which only one process may have, so run `franka_bringup` with
`load_gripper:=false` and nothing else.

**Know what it costs.** `blockedDuration(0, v)` is 363 ms, so the ceiling is 2.75 Hz and realistically
0.7–1.7 Hz — against a 10 Hz setpoint stream this node is a **decimator**, servicing roughly every
4th–15th setpoint. That is the hardware, not a bug. The `src/franka_hand_testing/` probes behind
those numbers build with `-DBUILD_HAND_PROBES=ON`. The binding constraint is the API — blocking
and un-pre-emptable — rather than the motor.

**It plans, it does not relay.** Setpoints land in a `WidthTrajectory` horizon, and `selectMove`
picks the earliest one still reachable, at the slowest speed that arrives on time. When nothing is
reachable it chases instead — full speed toward where the trajectory will be once the fingers get
there — because stopping short at an intermediate width costs another whole `fixed_cost`. On a
width plot that reads as aiming at the wrong target; it is not. All of it plans against
`HandLimits`: five numbers fitted from one hand, `fixed_cost` alone drifting 50–70 ms between runs,
every one a parameter — so re-run the probes and re-fit for your hand, not ours.

## Safety

This drives the arm by **torque**. On new hardware: e-stop in hand, start with the shipped gains,
and confirm the collision thresholds applied (configure fails loudly if they did not). Activation
itself is quiet — zero error means zero torque — but the arm is live from that moment.

MIT — see [LICENSE](LICENSE). [NOTICE](NOTICE) attributes the control law
(serl_franka_controllers) and the reference generator (UMI).
