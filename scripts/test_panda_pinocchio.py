import numpy as np
import pinocchio as pin

#model_path = '/workspace/assets/models/custom/panda_arm.urdf'
model_path = "/workspace/assets/models/franka_emika_panda/panda.xml"

model, *_ = pin.buildModelsFromMJCF(model_path)
data = model.createData()

q = pin.randomConfiguration(model)
print(q.T)