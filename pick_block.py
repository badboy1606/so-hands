
import numpy as np
import mujoco
import mujoco.viewer

MODEL_PATH = "so+hands/scene.xml"

ARM_JOINTS = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"]

FINGER_MOTORS = {
    1: ["finger1_motor1", "finger1_motor2"],
    2: ["finger2_motor1", "finger2_motor2"],
    3: ["finger3_motor1", "finger3_motor2"],
    4: ["finger4_motor1", "finger4_motor2"],
}
TIP_GEOMS  = {1: "tip1_col", 2: "tip2_col", 3: "tip3_col", 4: "tip4_col"}
BLOCK_GEOM = "block_geom"
EE_SITE    = "ee_site"

Q_INIT  = [1.5, -0.5,  0.5,  0.0, 0.0]
Q_ABOVE = [0.0, -2.5,  2.5,  0.5, 0.0]   # EE ≈ [0, -0.288, 0.145]  15 cm above block
Q_LOWER = [0.0, -2.0,  2.5,  0.5, 0.0]   # EE ≈ [0, -0.275, 0.041]  fingers at block
Q_LIFT  = [0.0, -2.5,  2.5,  0.0, 0.0]

OPEN_ANGLE = -0.3   # fingers open during descent
MAX_ANGLE  =  1.6   # max close angle — position controller presses at full force here

# Interpolation speeds (α units per sim-second)
ABOVE_SPEED  = 0.08   # slow swing to avoid knocking block
LOWER_SPEED  = 0.05   # gentle descent
SETTLE_SPEED = 0.25   # fast descent once 3+ fingers touched — seat block in arc
LIFT_SPEED   = 0.07   # slow rise so grip doesn't slip

MIN_CONTACT_FINGERS = 3   # require 3 sides before accelerating descent

# Grasp: fingers ramp from OPEN_ANGLE to MAX_ANGLE over CLOSE_SECS,
# then hold at MAX_ANGLE for HOLD_SECS so grip settles before lift.
CLOSE_SECS = 2.0   # time to fully curl fingers (smooth ramp, not a jump)
HOLD_SECS  = 1.0   # extra hold at max grip before lifting


def set_arm_ctrl(model, data, q):
    for name, val in zip(ARM_JOINTS, q):
        lo, hi = model.joint(name).range
        data.ctrl[model.actuator(name).id] = float(np.clip(val, lo, hi))


def set_all_fingers(model, data, angle):
    for f in range(1, 5):
        for motor in FINGER_MOTORS[f]:
            data.ctrl[model.actuator(motor).id] = angle


def touching_block(data, geom_id, block_gid):
    for i in range(data.ncon):
        c = data.contact[i]
        if (c.geom1 == geom_id and c.geom2 == block_gid) or \
           (c.geom2 == geom_id and c.geom1 == block_gid):
            return True
    return False


def count_finger_contacts(data, tip_gids, block_gid):
    return sum(touching_block(data, tip_gids[f], block_gid) for f in range(1, 5))


def main():
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data  = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)

    site_id   = model.site(EE_SITE).id
    block_bid = model.body("block").id
    block_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, BLOCK_GEOM)
    tip_gids  = {f: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, TIP_GEOMS[f])
                 for f in range(1, 5)}

    set_arm_ctrl(model, data, Q_INIT)
    for name, val in zip(ARM_JOINTS, Q_INIT):
        lo, hi = model.joint(name).range
        data.qpos[model.joint(name).qposadr[0]] = float(np.clip(val, lo, hi))
    set_all_fingers(model, data, OPEN_ANGLE)
    mujoco.mj_forward(model, data)
    for _ in range(500):
        mujoco.mj_step(model, data)

    print("=== Pick sequence ===")
    print(f"  Init EE  : {np.round(data.site_xpos[site_id], 3)}")
    print(f"  Block pos: {np.round(data.xpos[block_bid], 3)}")

    above_alpha  = [0.0]
    lower_alpha  = [0.0]
    lift_alpha   = [0.0]
    contact_made = [False]

    phases      = ["HOME", "ABOVE", "LOWER", "GRASP", "LIFT", "HOLD"]
    phase_idx   = 0
    phase_start = 0
    total_steps = 0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0.0, -0.25, 0.12]
        viewer.cam.distance  = 0.70
        viewer.cam.elevation = -15
        viewer.cam.azimuth   = 90

        while viewer.is_running():
            if phase_idx >= len(phases):
                mujoco.mj_step(model, data)
                viewer.sync()
                continue

            phase = phases[phase_idx]
            sip   = total_steps - phase_start
            dt    = model.opt.timestep
            adv   = False

            # ── HOME ─────────────────────────────────────────────────────────
            if phase == "HOME":
                set_arm_ctrl(model, data, Q_INIT)
                set_all_fingers(model, data, OPEN_ANGLE)
                if sip == 0:
                    print(f"\n[HOME] Settling...")
                adv = sip >= int(2.0 / dt)

            # ── ABOVE ────────────────────────────────────────────────────────
            # Slow interpolated swing so the arm never sweeps fast through
            # the block's position and knocks it away.
            elif phase == "ABOVE":
                above_alpha[0] = min(above_alpha[0] + ABOVE_SPEED * dt, 1.0)
                q_now = [a + above_alpha[0] * (b - a) for a, b in zip(Q_INIT, Q_ABOVE)]
                set_arm_ctrl(model, data, q_now)
                set_all_fingers(model, data, OPEN_ANGLE)
                if sip == 0:
                    print(f"\n[ABOVE] Slowly swinging above block...")
                if sip % 500 == 499:
                    ee = data.site_xpos[site_id]
                    print(f"  α={above_alpha[0]:.2f}  EE={np.round(ee, 3)}")
                adv = above_alpha[0] >= 1.0

            # ── LOWER ────────────────────────────────────────────────────────
            # Descend at LOWER_SPEED until 3+ fingers simultaneously touch
            # the block, then accelerate (SETTLE_SPEED) all the way to α=1.0
            # so the block sits fully inside the finger arc before grasping.
            elif phase == "LOWER":
                n_contact = count_finger_contacts(data, tip_gids, block_gid)

                if n_contact >= MIN_CONTACT_FINGERS and sip > 20:
                    if not contact_made[0]:
                        contact_made[0] = True
                        print(f"  → {n_contact} fingers on block at α={lower_alpha[0]:.2f} "
                              f"z={data.site_xpos[site_id][2]*100:.1f}cm — seating deeper...")
                    lower_alpha[0] = min(lower_alpha[0] + SETTLE_SPEED * dt, 1.0)
                else:
                    lower_alpha[0] = min(lower_alpha[0] + LOWER_SPEED * dt, 1.0)

                q_now = [a + lower_alpha[0] * (b - a) for a, b in zip(Q_ABOVE, Q_LOWER)]
                set_arm_ctrl(model, data, q_now)
                set_all_fingers(model, data, OPEN_ANGLE)

                if sip == 0:
                    print(f"\n[LOWER] Descending — need {MIN_CONTACT_FINGERS}+ fingers before seating...")
                if sip % 400 == 399:
                    ee = data.site_xpos[site_id]
                    print(f"  α={lower_alpha[0]:.2f}  EE={np.round(ee,3)}  fingers={n_contact}")

                if lower_alpha[0] >= 1.0:
                    print(f"  → Fully descended (α=1.0) — block seated  → GRASP")
                adv = lower_alpha[0] >= 1.0

            # ── GRASP ────────────────────────────────────────────────────────
            # Ramp all 4 fingers together from OPEN_ANGLE → MAX_ANGLE over
            # CLOSE_SECS, then hold for HOLD_SECS.
            # Smooth ramp lets fingers curl around the block sides rather than
            # jerking to target; equal targets on all 4 = symmetric pressure.
            elif phase == "GRASP":
                close_steps = int(CLOSE_SECS / dt)
                hold_steps  = int(HOLD_SECS  / dt)
                if sip < close_steps:
                    t_frac = sip / close_steps
                    target = OPEN_ANGLE + t_frac * (MAX_ANGLE - OPEN_ANGLE)
                else:
                    target = MAX_ANGLE
                set_all_fingers(model, data, target)

                if sip == 0:
                    ee = data.site_xpos[site_id]
                    bc = data.xpos[block_bid].copy()
                    print(f"\n[GRASP] Curling fingers ({CLOSE_SECS}s ramp → hold {HOLD_SECS}s)...")
                    print(f"  EE={np.round(ee,3)}  Block={np.round(bc,3)}")
                if sip % 400 == 399:
                    n  = count_finger_contacts(data, tip_gids, block_gid)
                    bz = data.xpos[block_bid][2]
                    print(f"  t={sip*dt:.1f}s  target={target:.2f}rad  "
                          f"fingers_on_block={n}  blockZ={bz*100:.1f}cm")

                adv = sip >= close_steps + hold_steps

            # ── LIFT ─────────────────────────────────────────────────────────
            # Slow interpolated rise from Q_LOWER → Q_LIFT.
            # Fingers held at MAX_ANGLE the whole way up.
            elif phase == "LIFT":
                lift_alpha[0] = min(lift_alpha[0] + LIFT_SPEED * dt, 1.0)
                q_now = [a + lift_alpha[0] * (b - a) for a, b in zip(Q_LOWER, Q_LIFT)]
                set_arm_ctrl(model, data, q_now)
                set_all_fingers(model, data, MAX_ANGLE)
                if sip == 0:
                    print(f"\n[LIFT] Slowly raising arm with block...")
                if sip % 400 == 399:
                    bz = data.xpos[block_bid][2]
                    ee = data.site_xpos[site_id]
                    print(f"  α={lift_alpha[0]:.2f}  EE={np.round(ee,3)}  blockZ={bz*100:.1f}cm")
                adv = lift_alpha[0] >= 1.0

            # ── HOLD ─────────────────────────────────────────────────────────
            elif phase == "HOLD":
                set_all_fingers(model, data, MAX_ANGLE)
                if sip == 0:
                    bz     = data.xpos[block_bid][2]
                    lifted = bz > 0.08
                    print(f"\n[HOLD] Block Z = {bz*100:.1f} cm")
                    print(f"  {'SUCCESS — block is lifted!' if lifted else 'Block not lifted.'}")
                adv = sip >= int(5.0 / dt)

            mujoco.mj_step(model, data)
            viewer.sync()
            total_steps += 1

            if adv:
                bz = data.xpos[block_bid][2]
                t  = total_steps * dt
                print(f"  → {phase} done  t={t:.1f}s  blockZ={bz*100:.1f}cm")
                phase_idx  += 1
                phase_start = total_steps
                if phase_idx < len(phases):
                    print(f"  → NEXT: {phases[phase_idx]}")

        print("Done.")


if __name__ == "__main__":
    main()
