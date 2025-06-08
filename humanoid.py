import pybullet as p
import pybullet_data
import time
import math

# Connect to GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0, 0, -9.8)

# Load plane and humanoid
plane_id = p.loadURDF("plane.urdf")
robots = p.loadMJCF(pybullet_data.getDataPath() + "/mjcf/humanoid.xml")

print("[humanoid] robots:", robots)
if not robots:
    print("[humanoid] Failed to load MJCF model!")
    exit()

robot_id = robots[1]
print("[humanoid] Humanoid robot loaded with ID:", robot_id)

# Let everything settle
for _ in range(100):
    p.stepSimulation()
    time.sleep(1./240.)

# Get joint info
num_joints = p.getNumJoints(robot_id)

for i in range(num_joints):
    info = p.getJointInfo(robot_id, i)
    # print(f"{i}: {info[1].decode('utf-8')}")


joint_names = [p.getJointInfo(robot_id, i)[1].decode('utf-8') for i in range(num_joints)]
joint_indices = {name: i for i, name in enumerate(joint_names)}

# Define leg joints to control
leg_joints = [
    "left_hip_x", "left_knee", "right_hip_x", "right_knee"
]

# Extract joint indices
controlled_joints = [joint_indices[name] for name in leg_joints]

# Set control mode to position
for j in controlled_joints:
    p.setJointMotorControl2(robot_id, j, controlMode=p.POSITION_CONTROL, force=100)

# Simulation loop
t = 0
while True:
    t += 0.01

    # Define walking pattern (simple sine waves)
    left_hip_angle  = 0.3 * math.sin(2 * math.pi * t)
    left_knee_angle = 0.6 * math.sin(2 * math.pi * t)
    right_hip_angle = 0.3 * math.sin(2 * math.pi * t + math.pi)
    right_knee_angle = 0.6 * math.sin(2 * math.pi * t + math.pi)

    # Apply joint positions
    target_positions = [left_hip_angle, left_knee_angle, right_hip_angle, right_knee_angle]

    for idx, target in zip(controlled_joints, target_positions):
        p.setJointMotorControl2(robot_id, idx, controlMode=p.POSITION_CONTROL,
                                targetPosition=target, force=100)

    p.stepSimulation()
    time.sleep(1. / 240.)
