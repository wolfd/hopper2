import pybullet as p
import time
import pybullet_data
import math

import numpy as np
import quaternion

# TL;DR: this is HOT garbage and hacked together from like 8 sources

physics_client = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, 0) #-9.81)
plane_id = p.loadURDF("plane.urdf")
cube_start_pos = [0, 0, 1]
cube_start_orientation = p.getQuaternionFromEuler([0, 1.2, 0])
# car_id = p.loadURDF("./models/gyro-car.urdf", cube_start_pos, cube_start_orientation)
car_id = p.loadURDF("./models/car.urdf", cube_start_pos, cube_start_orientation, useFixedBase=True)

wheel_indices = [0]

for joint_id in wheel_indices:
    p.changeDynamics(car_id, joint_id, linearDamping=0, angularDamping=0)

p.setJointMotorControlArray(car_id, wheel_indices, p.VELOCITY_CONTROL, forces=[0.0])

def getJointStates(robot):
	joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
	joint_positions = [state[0] for state in joint_states]
	joint_velocities = [state[1] for state in joint_states]
	joint_torques = [state[3] for state in joint_states]
	return joint_positions, joint_velocities, joint_torques

def getMotorJointStates(robot):
	joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
	joint_infos = [p.getJointInfo(robot, i) for i in range(p.getNumJoints(robot))]
	joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
	joint_positions = [state[0] for state in joint_states]
	joint_velocities = [state[1] for state in joint_states]
	joint_torques = [state[3] for state in joint_states]
	return joint_positions, joint_velocities, joint_torques

def setJointPosition(robot, position, kp=1.0, kv=0.3):
	num_joints = p.getNumJoints(robot)
	zero_vec = [0.0] * num_joints
	if len(position) == num_joints:
		p.setJointMotorControlArray(robot, range(num_joints), p.POSITION_CONTROL,
			targetPositions=position, targetVelocities=zero_vec,
			positionGains=[kp] * num_joints, velocityGains=[kv] * num_joints)
	else:
		print("Not setting torque. "
			  "Expected torque vector of "
			  "length {}, got {}".format(num_joints, len(torque)))

def multiplyJacobian(robot, jacobian, vector):
	result = [0.0, 0.0, 0.0]
	i = 0
	for c in range(len(vector)):
		if p.getJointInfo(robot, c)[3] > -1:
			for r in range(3):
				result[r] += jacobian[r][i] * vector[c]
			i += 1
	return result


def attitude_error():
    car_pos, car_orientation = p.getBasePositionAndOrientation(car_id)

    att = np.quaternion(*car_orientation)

    err_x, err_y, err_z = (0 - v for v in p.getEulerFromQuaternion(car_orientation))

    print(err_x, err_y, err_z)

    # desired angle at time -- determined by -- wheels spun down before hitting ground


    # set up linear equation
    # desired body angular velocity (x, y, z) = jacobians_rot *
    # set_motor()



# seconds
def stabilization_control(time_horizon = 2.0):
    print('dynamics: ' + str(p.getDynamicsInfo(car_id, -1)))
    joints = len(p.getJointStates(car_id, wheel_indices))
    print('joints: ' + str(joints))
    print('numJoints: ' + str(p.getNumJoints(car_id)))

    # import ipdb; ipdb.set_trace()
    # for dof in range(0, 16):
    #     target_angle = [0.0] * dof
    #     target_velocity = [0.0] * dof
    #     target_accel = [0.0] * dof
    #     try:
    #         torque = p.calculateInverseDynamics(car_id, target_angle, target_velocity, target_accel)
    #         print("worked with {}".format(dof))
    #     except SystemError as e:
    #         pass # print(e)

    # setJointPosition(car_id, [0.1])
    p.stepSimulation()

    pos, vel, torq = getJointStates(car_id)

    mpos, mvel, mtorq = getMotorJointStates(car_id)

    result = p.getLinkState(car_id, 0, computeLinkVelocity=1, computeForwardKinematics=1)
    link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result

    print('numJoints: {}'.format(p.getNumJoints(car_id)))
    vehicle_com_relative_to_first_wheel = [-0.1015, -0.09255, 0]
    zero_vec = [0.0] * len(mpos)
    for i in range(p.getNumJoints(car_id)):
        wheel_jacobian = np.array(p.calculateJacobian(car_id, i, com_trn, mpos, zero_vec, zero_vec))

        wheel_j_trans_I = np.linalg.pinv(wheel_jacobian[0])
        wheel_j_rot_I = np.linalg.pinv(wheel_jacobian[1])

        print(wheel_j_rot_I)


    # import ipdb; ipdb.set_trace()

    # print("link angular velocity of () from ")
    # print(multiplyJacobian(car_id, wheel_jacobian[1], vel))

    torque = [100]

    return torque

def calculate_approx_motor_torque():
    I = 0.000132651 # approx izz for wheel
    a = 25132 # rads/s^2 (assuming 1s spinup time to 4000rpm, which is VERY conservative)
    a = 101000 # 4000 * 2pi rad/s /  0.25s in rads/s^2
    T = I * a
    return T # N * m

def set_motor(wheel_index, target_speed):
    p.setJointMotorControl2(
        bodyUniqueId=car_id,
        # front left, front right, back left, back right
        jointIndex=wheel_index,
        force=calculate_approx_motor_torque(),
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=front_speed
    )

while True:
    # p.stepSimulation()

    car_pos, car_orientation = p.getBasePositionAndOrientation(car_id)

    torques = stabilization_control()

    test = [0, 0, 0, 0]
    print('test')
    print(p.calculateInverseDynamics(car_id, test, test, test))

    attitude_error()

    # p.setJointMotorControl2(
    #     bodyUniqueId=car_id,
    #     jointIndex=0, #  front left
    #     controlMode=p.VELOCITY_CONTROL,
    #     targetVelocity=100
    # )

    # front_speed, back_speed = gyro_control_update(math.degrees(p.getEulerFromQuaternion(car_orientation)[1]))
    # print(p.getEulerFromQuaternion(car_orientation))
    # front_speed = 0#100
    # back_speed = 100
    # set_motors(front_speed, back_speed)

    time.sleep(1/240.)

cube_pos, cube_orientation = p.getBasePositionAndOrientation(car_id)

print(cube_pos, cube_orientation)

p.disconnect()
