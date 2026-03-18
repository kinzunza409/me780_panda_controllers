import numpy as np

import mujoco
import mujoco.viewer
from dm_control import mjcf

n_joints = 7 # only manipulator, no gripper
Kp = 500*np.ones(n_joints)
Kd = 100*np.ones(n_joints)

qd = np.ones(n_joints)
qd[3] = -1
qdot_d = np.zeros(n_joints)

# returns joint torques
def KP_control(q, qd, qdot, qdot_d):
    tau = Kp * (qd - q) + Kd * (qdot_d - qdot)
    return tau

def main():
    
    root = mjcf.from_path("/workspace/assets/models/franka_emika_panda/scene.xml")
    
    # change model to direct torque control
    for actuator in root.find_all('actuator'):
        actuator.remove()

    # Add motor actuators for arm joints only
    root.actuator.add('motor', name='actuator1', joint='joint1', gear=[1], ctrllimited=True, ctrlrange=[-87, 87])
    root.actuator.add('motor', name='actuator2', joint='joint2', gear=[1], ctrllimited=True, ctrlrange=[-87, 87])
    root.actuator.add('motor', name='actuator3', joint='joint3', gear=[1], ctrllimited=True, ctrlrange=[-87, 87])
    root.actuator.add('motor', name='actuator4', joint='joint4', gear=[1], ctrllimited=True, ctrlrange=[-87, 87])
    root.actuator.add('motor', name='actuator5', joint='joint5', gear=[1], ctrllimited=True, ctrlrange=[-12, 12])
    root.actuator.add('motor', name='actuator6', joint='joint6', gear=[1], ctrllimited=True, ctrlrange=[-12, 12])
    root.actuator.add('motor', name='actuator7', joint='joint7', gear=[1], ctrllimited=True, ctrlrange=[-12, 12])
    root.actuator.add('motor', name='actuator8', tendon='split', gear=[1], ctrllimited=True, ctrlrange=[-100, 100])

    physics = mjcf.Physics.from_mjcf_model(root)
    model = physics.model.ptr
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            tau = KP_control(
                q = data.qpos[:n_joints],
                qd = qd,
                qdot = data.qvel[:n_joints],
                qdot_d = qdot_d)
            
            data.ctrl[:n_joints] = tau

            mujoco.mj_step(model, data)
            viewer.sync()


if __name__=='__main__':
    main()