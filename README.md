# franka_streaming_impedance_controller

A streaming Cartesian impedance controller for the Franka FR3, driven by **absolutely-timed pose
chunks**. It splices consecutive chunks on their own waypoint times, so the arm neither starts
from rest nor stops at a chunk boundary — a chunked reference (a learned policy's action chunks,
a planner's sampled path) keeps its timeline all the way down to joint torques.

Built for contact-rich work: the impedance is a real Cartesian mass-spring-damper with a bounded
commanded force, and the control point is any TF frame you name, not just franka's own tool frame.

Two packages:

| Package | Build type | What it is |
|---|---|---|
| `franka_streaming_impedance_controller` | `ament_cmake` | The ros2_control plugin, the control law and interpolators as a separately testable library, and `franka_hand_node` |
| `franka_streaming_impedance_client` | `ament_python` | Producer side: builds the timed `MultiDOFJointTrajectory` chunks the controller consumes |

## Requirements

`franka_ros2` (`franka_hardware`, `franka_semantic_components`, `franka_msgs`) and `libfranka`,
plus a realtime kernel — this is a 1 kHz torque controller. Developed against ROS 2 Humble on the
control PC; the Python client is distro-agnostic.

## Build

```bash
# into a workspace that already has franka_ros2 + libfranka
ln -s /path/to/franka_streaming_impedance_controller/* src/
colcon build --packages-select franka_streaming_impedance_controller franka_streaming_impedance_client \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select franka_streaming_impedance_controller   # 59 gtests, no hardware needed
```

The control law and interpolators build as `franka_streaming_impedance_core`, a library with no
hardware interface, which is why they are unit-testable at all.

## Running it

Load the controller **inactive**, then switch to it — it claims the same `<joint>/effort`
interfaces as your joint-trajectory controller, so exactly one of the two can hold the arm:

```bash
ros2 run controller_manager spawner cartesian_impedance_controller \
  -t franka_streaming_impedance_controller/CartesianImpedanceController \
  --param-file config/controllers.example.yaml --inactive

ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: ['cartesian_impedance_controller'], \
    deactivate_controllers: ['fr3_arm_controller'], strictness: 2}"
```

Switching restarts the libfranka control loop, so **only switch with the arm stationary**.

## The wire contract

The controller subscribes to `target_topic` as `trajectory_msgs/MultiDOFJointTrajectory`. Three
rules, each of which makes it reject the chunk rather than guess:

- **`header.frame_id` must equal the controller's `base_frame`.** Poses are commanded in the base
  frame.
- **`joint_names` must be exactly `[tcp_frame]`.** It names the *commanded frame*, not a joint. A
  chunk addressed to a different frame is not reinterpreted just because it landed on this topic.
- **Waypoint times are absolute**: `header.stamp + time_from_start`. This is what makes splicing
  work — a new chunk overwrites the old one only from its own first waypoint onward, and whatever
  is still pending before that keeps playing.

Publishing ahead of time is how you compensate action latency: stamp a chunk in the past and its
early waypoints are simply already due.

If you slice stale waypoints off the front of a chunk before publishing, keep numbering from the
*unsliced* index (`first_index` in the client). Renumbering the survivors from zero shifts the
whole timeline earlier, which looks like tracking lag rather than a bug.

### Producing chunks

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

`topic` is deliberately required: the controller's own default is node-relative, so a client-side
default would resolve against the *publishing* node and silently address nothing.
`get_subscription_count()` is exposed because publishing where nothing subscribes is otherwise
completely silent — nothing moves and no error appears anywhere.

## Parameters

| Parameter | Default | Notes |
|---|---|---|
| `arm_id` | `fr3` | Drives every `<arm_id>_*` joint and frame name |
| `base_frame` | `fr3_link0` | Frame chunks are commanded in |
| `tcp_frame` | `fr3_hand_tcp` | **The control point.** Any TF frame rigidly attached to the flange |
| `target_topic` | `~/target_poses` | Node-relative by default |
| `translational_stiffness` / `_damping` | 2000.0 / 89.0 | N/m; damping ~critical at `2*sqrt(k)` |
| `rotational_stiffness` / `_damping` | 150.0 / 7.0 | Deliberately under-damped, as SERL and UMI both run it |
| `translational_clip` / `rotational_clip` | 0.01 / 0.05 | m / rad — **this is the force ceiling**, see below |
| `translational_ki` / `rotational_ki` | 0.0 | Off; for steady-state error under sustained load |
| `nullspace_stiffness` | 0.2 | Resolves the 7th DOF |
| `joint1_nullspace_stiffness` | 100.0 | Joint 1 pinned much harder, so the base holds while the elbow floats |
| `max_pos_speed` / `max_rot_speed` | 1.0 / 3.14 | Caps how fast the equilibrium point travels, i.e. how far it leads the arm |
| `collision.*` | franka's `DefaultRobotBehavior` | Applied at configure; **configure fails if they do not take** |

### The one number that matters: stiffness × clip

Commanded force is `stiffness * error`, and the error is clipped per axis — so the clip *is* the
force ceiling. At the defaults that is `2000 N/m × 0.01 m = 20 N`. Raising `translational_clip`
raises the ceiling proportionally, and it is the single knob to turn if the arm feels too weak
against a surface.

This is why the stiffness is high rather than UMI's softer 750 N/m: a high stiffness with a tight
clip tracks accurately *and* saturates force. A soft unbounded spring instead leans on the
environment with everything the arm has when the policy commands an unreachable pose.

**Collision thresholds are not an independent choice.** The clip bounds *commanded* force; the
reflex fires on *estimated external* force, which includes whatever the environment pushes back
with. Raise the clip and these have to follow, or the arm reflexes exactly when it starts doing
useful work.

### Why a custom `tcp_frame`

Franka reports `O_T_EE` at `<arm_id>_hand_tcp`. If your actual contact point is elsewhere — custom
fingers, a tool — a spring anchored at the wrong point turns an orientation error into a large
translation at the real contact point. Both the measured pose and the Jacobian are moved onto
`tcp_frame` at activation with a single TF lookup, so whatever publishes that transform stays the
only place your tool geometry is written down.

Note `franka_msgs/srv/SetTCPFrame` (libfranka `setEE`) is **not** the lever here — it changes
`O_T_EE` reporting only, while TF and MoveIt are driven by the URDF.

### Why torque, not `cartesian_pose`

`franka_hardware`'s native `cartesian_pose` interface looks like the direct analogue of UMI's
`update_desired_ee_pose`, but under it libfranka hardcodes `ControllerMode::kJointImpedance`,
whose gains are stiffness-only with no damping knob at all. Contact tasks need a real Cartesian
mass-spring-damper. That interface also has its rate limiter and low-pass filter hardcoded off, so
nothing downstream catches a discontinuity either way — which is why the interpolator is mandatory
and activation always seeds from the measured pose.

## `franka_hand_node`

An optional driver for a stock Franka Hand, consuming `trajectory_msgs/JointTrajectory` widths in
metres on `target_topic` (default `~/target_widths`) and republishing finger state on
`~/joint_states`.

It talks to libfranka **directly**, not through `franka_gripper`, because a `Move` cannot be
pre-empted: `stop()` queues *behind* it, and a superseding `Move` stops the fingers rather than
redirecting them. Owning the connection makes run-to-completion structural — exactly one thread
calls `move()`, in a loop.

**Know what it costs.** `blockedDuration(0)` is 363 ms, so the command ceiling is 2.75 Hz and
realistically 0.7–1.7 Hz. Against a 10 Hz setpoint stream this node is a **decimator**, servicing
roughly every 4th–15th setpoint. That is the hardware, not a bug. It also holds the libfranka
gripper connection, which only one process may have — so run `franka_bringup` with
`load_gripper:=false` and do not run `franka_gripper` alongside it.

The `src/franka_hand_testing/` probes that characterised those numbers are off by default; build
them with `--cmake-args -DBUILD_HAND_PROBES=ON` to re-characterise a different hand.

## Safety

This drives the arm by **torque**. Before a first run on new hardware: keep the e-stop in hand,
start with the shipped gains, and verify the collision thresholds actually applied (configure
fails loudly if they did not). The controller commands zero torque on a zero error at activation,
so activation itself is quiet — but it is live from that moment.

## License

MIT. See [LICENSE](LICENSE), which also carries the attribution for the control law
(serl_franka_controllers) and the reference generator (UMI).
