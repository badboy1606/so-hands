import mujoco
import mujoco.viewer

# Load the model
model = mujoco.MjModel.from_xml_path("trs_so_arm100/scene.xml")
data = mujoco.MjData(model)

# Launch interactive viewer
mujoco.viewer.launch(model, data)