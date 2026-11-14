"""Control: DroneState + Setpoint -> RCFrame (P1).

Horizontal: a = acc_ref + Kp(p_ref - p) + Kd(v_ref - v) - bias ; tilt = atan(a/g),
rotate world->body with camera yaw, map to sticks with the identified gain.
Vertical: climb-rate = z_ref_dot + Kz(z_ref - z), sigma-delta dither through the
+-40 us deadband. Plant numbers: config/plant/pluto_v5.yaml.
"""
