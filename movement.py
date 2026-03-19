import mujoco
import mujoco.viewer
import numpy as np
import time

# Load model
model = mujoco.MjModel.from_xml_path("so+hands/scene1.xml")
data = mujoco.MjData(model)

print("Number of actuators:", model.nu)

t = 0.0

with mujoco.viewer.launch_passive(model, data) as viewer:

    start = time.time()

    while viewer.is_running():

        t = time.time() - start

        # ===== ARM MOTION =====
        data.ctrl[0] = 0.5 * np.sin(t)          # base rotation
        data.ctrl[1] = -1.5 + 0.3*np.sin(t)     # shoulder
        data.ctrl[2] = 1.5 + 0.3*np.cos(t)      # elbow
        data.ctrl[3] = 0.5*np.sin(0.5*t)        # wrist pitch
        data.ctrl[4] = 0.5*np.cos(0.5*t)        # wrist roll

        # ===== HAND OPEN/CLOSE =====
        grip = 0.8*np.sin(1.5*t)

        data.ctrl[5]  = grip
        data.ctrl[6]  = -grip

        data.ctrl[7]  = grip
        data.ctrl[8]  = -grip

        data.ctrl[9]  = grip
        data.ctrl[10] = -grip

        data.ctrl[11] = grip
        data.ctrl[12] = -grip

        mujoco.mj_step(model, data)

        viewer.sync()