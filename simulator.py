import pybullet as p
import time
import pybullet_data
import math

physics_client = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, 0) #-9.81)
plane_id = p.loadURDF("plane.urdf")
cube_start_pos = [0, 0, 1]
cube_start_orientation = p.getQuaternionFromEuler([0, 1.2, 0])
# car_id = p.loadURDF("./models/gyro-car.urdf", cube_start_pos, cube_start_orientation)
car_id = p.loadURDF("./models/car.urdf", cube_start_pos, cube_start_orientation)

angle_error_accum = 0

def gyro_control_update(x_angle):
    global angle_error_accum
    # Control parameters
    kp = 1
    ki = 1
    jsig = 1.36
    angle_des = 0.0

    # Other local variables
    s = 0                       # empty speed holder
    err = angle_des - x_angle  # angle error term
    front_sig = 0  # empty front wheel signal holder
    rear_sig = 0   # empty rear wheel signal holder

    # Update global error accumulator
    angle_error_accum += err

    # If the angle change is beyond a threshold, calculate wheel speeds
    if abs(x_angle) > 3:
        s = kp*err + ki*angle_error_accum
    else:
        # If the angle is small enough to be neglegible/noise, do nothing
        s = 0

    # Now let's generate the signal for each wheel
    if x_angle == 0:
        front_sig = 0
        rear_sig = 0
    elif x_angle > 0:
        # If the angle is positive we're tilted nose up; spin front wheels backward
        front_sig = -jsig*s
        rear_sig = jsig*s
    elif x_angle < 0:
        # If the angle is negative we're tilted nose down; spin front wheels forwards
        front_sig = jsig*s
        rear_sig = -jsig*s

    # Set front wheel speeds
    # cmd1 = front_sig
    # cmd2 = front_sig
    # Serial.print("Front : ")
    # Serial.print(front_sig)

    # Set back wheel speeds
    # cmd3 = rear_sig
    # cmd4 = rear_sig
    # Serial.print("\tRear : ")
    # Serial.print(rear_sig)

    return (front_sig, rear_sig)


def set_motors(front_speed, back_speed):
    p.setJointMotorControl2(
        bodyUniqueId=car_id,
        jointIndex=0, #  front left
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=front_speed
    )

    p.setJointMotorControl2(
        bodyUniqueId=car_id,
        jointIndex=1, #  front right
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=front_speed
    )

    p.setJointMotorControl2(
        bodyUniqueId=car_id,
        jointIndex=2, #  back left
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=back_speed
    )

    p.setJointMotorControl2(
        bodyUniqueId=car_id,
        jointIndex=3, #  back right
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=back_speed
    )

for i in range(10000):
    p.stepSimulation()

    car_pos, car_orientation = p.getBasePositionAndOrientation(car_id)
    # front_speed, back_speed = gyro_control_update(math.degrees(p.getEulerFromQuaternion(car_orientation)[1]))
    print(p.getEulerFromQuaternion(car_orientation))
    front_speed = 0#100
    back_speed = 100
    set_motors(front_speed, back_speed)

    time.sleep(1/240.)

cube_pos, cube_orientation = p.getBasePositionAndOrientation(car_id)

print(cube_pos, cube_orientation)

p.disconnect()
