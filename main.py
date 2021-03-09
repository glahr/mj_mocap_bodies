from mujoco_py import load_model_from_path, MjSim, MjViewer
import numpy as np

model = load_model_from_path("/home/glahr/.mujoco/mujoco200/model/humanoid_doctor.xml")

sim = MjSim(model)
viewer = MjViewer(sim)

n_timesteps = 2000

t = 0

while t < n_timesteps:
    # read_joints = function_vision()

    # sim.data.set_joint_qpos('right_shoulder1', qpos_desired)
    # sim.data.set_joint_qpos('right_shoulder2', qpos_desired)
    # sim.data.set_joint_qpos('right_elbow', qpos_desired)
    # sim.data.set_joint_qpos('left_shoulder1', qpos_desired)
    # sim.data.set_joint_qpos('left_shoulder2', qpos_desired)
    # sim.data.set_joint_qpos('left_elbow', qpos_desired)

    sim.data.set_joint_qpos('right_elbow', sim.data.get_joint_qpos('right_elbow') + 0.05*np.sin(0.01*t))
    sim.forward()
    viewer.render()
    t += 1