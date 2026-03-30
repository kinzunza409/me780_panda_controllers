import mujoco
from dm_control import mjcf
import os

def model_to_torque_control(input_path, output_path):
    root = mjcf.from_path(input_path)

    input_dir = os.path.abspath(os.path.dirname(input_path))
    root.compiler.meshdir = input_dir

    for actuator in root.find_all('actuator'):
        actuator.remove()

    root.actuator.add('motor', name='actuator1', joint='joint1', gear=[1], ctrllimited=True, ctrlrange=[-87, 87])
    root.actuator.add('motor', name='actuator2', joint='joint2', gear=[1], ctrllimited=True, ctrlrange=[-87, 87])
    root.actuator.add('motor', name='actuator3', joint='joint3', gear=[1], ctrllimited=True, ctrlrange=[-87, 87])
    root.actuator.add('motor', name='actuator4', joint='joint4', gear=[1], ctrllimited=True, ctrlrange=[-87, 87])
    root.actuator.add('motor', name='actuator5', joint='joint5', gear=[1], ctrllimited=True, ctrlrange=[-12, 12])
    root.actuator.add('motor', name='actuator6', joint='joint6', gear=[1], ctrllimited=True, ctrlrange=[-12, 12])
    root.actuator.add('motor', name='actuator7', joint='joint7', gear=[1], ctrllimited=True, ctrlrange=[-12, 12])
    root.actuator.add('motor', name='actuator8', tendon='split', gear=[1], ctrllimited=True, ctrlrange=[-100, 100])

    # Export with assets — dm_control bundles all mesh bytes into this dict
    xml_string, assets = root.to_xml_string(), root.get_assets()

    # Validate in-memory using the assets dict so MuJoCo doesn't need to hit disk
    mujoco.MjModel.from_xml_string(xml_string, assets=assets)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w') as f:
        f.write(xml_string)

    # Also write all mesh assets alongside the output XML
    output_dir = os.path.dirname(output_path)
    for filename, data in assets.items():
        asset_path = os.path.join(output_dir, filename)
        with open(asset_path, 'wb') as f:
            f.write(data)

    print(f"Written to {output_path} with {len(assets)} asset(s)")

if __name__ == '__main__':
    model_to_torque_control(
        input_path="/workspace/assets/models/franka_emika_panda/scene.xml",
        output_path="/workspace/assets/models/custom/scene_torque.xml"
    )