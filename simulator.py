import pybullet as p
import time
import pybullet_data

physics_client = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0,0,-10)
plane_id = p.loadURDF("plane.urdf")
cube_start_pos = [0,0,1]
cube_start_orientation = p.getQuaternionFromEuler([0,0,0])
car_id = p.loadURDF("./models/gyro-car.urdf", cube_start_pos, cube_start_orientation)

for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)

cube_pos, cube_orientation = p.getBasePositionAndOrientation(car_id)

print(cube_pos, cube_orientation)

p.disconnect()
