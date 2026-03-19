import mujoco
import mujoco.viewer

# Load the model
model = mujoco.MjModel.from_xml_path("so+hands/scene1.xml")
data = mujoco.MjData(model)

# Launch interactive viewer
mujoco.viewer.launch(model, data)