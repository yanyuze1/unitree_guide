# PlotJuggler Topic Guide For StateTrotting Yaw Debug

Use these topic groups as separate PlotJuggler panels when the robot falls during yaw commands.

1. Yaw control chain
- `/unitree_guide/debug/trotting/dyaw_cmd`
- `/unitree_guide/debug/trotting/dyaw_est`
- `/unitree_guide/debug/trotting/yaw_cmd`
- `/unitree_guide/debug/trotting/yaw_est`
- `/unitree_guide/debug/trotting/yaw_error`
- `/unitree_guide/debug/trotting/yaw_rate_mode`
- `/unitree_guide/debug/trotting/yaw_hold_deadband`
- `/unitree_guide/debug/trotting/rot_error_raw`
- `/unitree_guide/debug/trotting/rot_error`
- `/unitree_guide/debug/trotting/d_wbd`

2. Gait timing and contact state
- `/unitree_guide/debug/trotting/step_request`
- `/unitree_guide/debug/trotting/wave_status`
- `/unitree_guide/debug/trotting/phase`
- `/unitree_guide/debug/trotting/contact`

3. Force distribution and solver margin
- `/unitree_guide/debug/balance/min_constraint_margin`
- `/unitree_guide/debug/balance/normal_force`
- `/unitree_guide/debug/balance/force_error`
- `/unitree_guide/debug/balance/torque_error`
- `/unitree_guide/debug/trotting/force_feet_global`
- `/unitree_guide/debug/trotting/force_feet_body`

4. Body tracking
- `/unitree_guide/debug/trotting/pos_error`
- `/unitree_guide/debug/trotting/vel_error`
- `/unitree_guide/debug/trotting/dd_pcd`
- `/unitree_guide/debug/trotting/gyro_global`
- `/unitree_guide/debug/estimator/gyro_global`

5. Joint side effects when the fall starts
- `/unitree_guide/debug/joint/tau_cmd`
- `/unitree_guide/debug/joint/tau_state`
- `/unitree_guide/debug/joint/tau_error`
- `/unitree_guide/debug/joint/q_error`
- `/unitree_guide/debug/joint/qd_error`

What to look for:
- If `yaw_rate_mode` becomes `1` while `rot_error_raw.z` is still large, compare it against `rot_error.z` and `d_wbd.z`.
- If `step_request` toggles near the fall, align it with `phase`, `contact`, and each leg's normal force.
- If `min_constraint_margin` goes negative or quickly approaches zero, the balance QP is losing feasible contact force margin.
