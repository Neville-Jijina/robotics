"""lab6_controller_1 controller."""
import numpy as np
import sys

try:
    from controller import Supervisor
except ImportError:
    print("ERROR: Must run inside Webots as Supervisor."); sys.exit(1)


approach_height = 0.15
grasp_height = 0.0
place_height = 0.0

# ============================================================
# Robot Interface (DO NOT MODIFY)
# ============================================================
class UR5eInterface:
    MOTOR_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                   "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    SENSOR_NAMES = [n + "_sensor" for n in MOTOR_NAMES]
    HAND_MOTORS_1 = ["finger_1_joint_1", "finger_2_joint_1", "finger_middle_joint_1"]
    HAND_MOTORS_2 = ["finger_1_joint_2", "finger_2_joint_2", "finger_middle_joint_2"]

    def __init__(self, robot):
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())
        self.motors, self.sensors = [], []
        for i in range(6):
            m = robot.getDevice(self.MOTOR_NAMES[i])
            s = robot.getDevice(self.SENSOR_NAMES[i])
            s.enable(self.timestep); m.setVelocity(1.0)
            self.motors.append(m); self.sensors.append(s)
        self.hand_motors_1, self.hand_motors_2 = [], []
        for name in self.HAND_MOTORS_1:
            m = robot.getDevice(name)
            if m: self.hand_motors_1.append(m)
        for name in self.HAND_MOTORS_2:
            m = robot.getDevice(name)
            if m: self.hand_motors_2.append(m)
        for _ in range(10): robot.step(self.timestep)

    def get_joint_positions(self):
        return np.array([s.getValue() for s in self.sensors])

    def set_joint_positions(self, q):
        for i in range(6): self.motors[i].setPosition(float(q[i]))

    def set_speed(self, speed):
        for m in self.motors: m.setVelocity(speed)

    def open_gripper(self):
        for m in self.hand_motors_1: m.setPosition(0.05)
        for m in self.hand_motors_2: m.setPosition(0.0)

    def close_gripper(self):
        for m in self.hand_motors_1: m.setPosition(0.3)
        for m in self.hand_motors_2: m.setPosition(0.8)

    def wait(self, seconds):
        for _ in range(int(seconds * 1000 / self.timestep)):
            if self.robot.step(self.timestep) == -1: return

    def move_to(self, q, wait_time=3.0):
        self.set_joint_positions(q); self.wait(wait_time)
        return self.get_joint_positions()


# ============================================================
# Constants (DO NOT MODIFY)
# ============================================================
# UR5e Standard DH Parameters
UR5E_DH_A     = [0.0, -0.425, -0.3922, 0.0, 0.0, 0.0]
UR5E_DH_D     = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]
UR5E_DH_ALPHA = [np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0]
GRIPPER_OFFSET = 0.18  # Robotiq 3F gripper length

# Webots UR5e PROTO has a 180-deg Z rotation between PROTO frame and DH frame
R_PROTO = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
ROBOT_POS = None  # Set at runtime from Supervisor

HOME = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]


# ============================================================
# PROVIDED: Coordinate Transforms (DO NOT MODIFY)
# ============================================================
def _dh(a, d, alpha, theta):
    """Standard DH transformation matrix (one joint)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ]])


def world_to_base(pw):
    """Convert world position to DH base frame."""
    return R_PROTO.T @ (np.array(pw) - ROBOT_POS)


def base_to_world(pb):
    """Convert DH base frame position to world."""
    return R_PROTO @ np.array(pb) + ROBOT_POS


# ============================================================
# TASK 1: Forward Kinematics (15 pts)
# ============================================================
def forward_kinematics(q):
    """
    Compute the gripper tip position given joint angles.

    Parameters:
        q : array-like, 6 joint angles (radians)

    Returns:
        position : numpy array [x, y, z] in robot base frame
    """
    # Needs to take in 6 joint angles and return the position of the gripper tip in the robot base frame. 
    T = np.eye(4)

    for i in range(6):
        a = UR5E_DH_A[i]
        b = UR5E_DH_D[i]
        alpha = UR5E_DH_ALPHA[i]
        theta = q[i]
        T = T @ _dh(a, b, alpha, theta)

    T_tool = np.eye(4)
    T_tool[2, 3] = GRIPPER_OFFSET
    T = T @ T_tool

    return T[0:3, 3]

    # print("testing testing testsing")
    # raise NotImplementedError("TODO: Implement forward_kinematics()")


# ============================================================
# TASK 2: Numerical Jacobian (15 pts)
# ============================================================
def compute_jacobian(q, delta=1e-5):
    """
    Compute the 3x6 position Jacobian using central finite differences.

    Parameters:
        q     : numpy array (6,) current joint angles
        delta : perturbation size

    Returns:
        J : numpy array (3, 6)
    """

    q = np.array(q, dtype=float)
    J = np.zeros((3, 6))

    for i in range(6):
        q_plus = q.copy()
        q_minus = q.copy()
        q_plus[i] += delta
        q_minus[i] -= delta

        f_plus = forward_kinematics(q_plus)
        f_minus = forward_kinematics(q_minus)

        J[:, i] = (f_plus - f_minus) / (2 * delta)

    return J

    # TODO: Implement (~8-10 lines)
    # raise NotImplementedError("TODO: Implement compute_jacobian()")


# ============================================================
# TASK 3: Define Waypoints (10 pts)
# ============================================================
def get_waypoints(block_world, bin_world):
    """
    Define the world-frame positions the gripper should visit.

    Parameters:
        block_world : [x, y, z] of block center in world frame
        bin_world   : [x, y, z] of bin center in world frame

    Returns:
        dict with keys: 'above_block', 'at_block', 'above_bin'
        Each value is a list [x, y, z] in world frame.
    """
    # raise NotImplementedError("TODO: Implement get_waypoints()")

    # Above Block (0.3, -0.15, 0.84)
    # Above Box (0.3, 0.2, 0.84)

    block = np.array(block_world, dtype= float)
    bin_pos = np.array(bin_world, dtype= float)

    up = np.array([0.0, 0.0, 1.0])

    waypoints = {
        "pre_grasp": block + approach_height * up,
        "grasp": block + grasp_height * up,
        "lift": block + approach_height * up,
        "pre_place": bin_pos + approach_height * up,
        "place": bin_pos + place_height * up,
        "return": bin_pos + approach_height * up
    }

    return waypoints

# ============================================================
# TASK 4: Gradient Descent IK (20 pts)
# ============================================================
def gradient_descent_ik(target_world, q_init, learning_rate, max_iterations, tolerance):
    """
    Find joint angles that place the gripper at target_world.

    Parameters:
        target_world   : [x, y, z] desired position in world frame
        q_init         : starting joint angles (numpy array of 6)
        learning_rate  : step size (default 0.5)
        max_iterations : max steps (default 1000)
        tolerance      : converge when error < this (meters)

    Returns:
        q_solution : numpy array of 6 joint angles
        converged  : bool
        errors     : list of position error at each iteration
    """
    # raise NotImplementedError("TODO: Implement gradient_descent_ik()")

    target_base = np.array(world_to_base(target_world), dtype=float)

    q = np.array(q_init, dtype=float)

    for i in range(max_iterations):
        current = forward_kinematics(q)
        error = target_base - current

        if np.linalg.norm(error) < tolerance:
            return q

        J = compute_jacobian(q)

        q = q + learning_rate * (J.T @ error)

        q = np.clip(q, -2 * np.pi, 2 * np.pi)

    print("IK error: ", np.linalg.norm(error))
    return q




# ============================================================
# TASK 5: Pick and Place Sequence (10 pts)
# ============================================================
def pick_and_place(arm, block_world, bin_world):
    """
    Pick up the block and place it in the bin.

    """
    # TODO: Implement 
    # raise NotImplementedError("TODO: Implement pick_and_place()")

    wp = get_waypoints(block_world, bin_world)
    print("waypoint keys:", list(wp.keys()))

    arm.open_gripper()
    arm.move_to(HOME)
    q = np.array(HOME, dtype= float)

    plan = {
        ("pre_grasp", 'open'),
        ("grasp", 'close'),
        ("lift", None),
        ("pre_place", None),
        ("place", 'open'),
        ("return", None),
    }

    for name, gripper_action in plan:
        q = gradient_descent_ik(wp[name], q, learning_rate= 0.5, max_iterations= 5000, tolerance= 1e-3)
        reached = base_to_world(forward_kinematics(q))
        err = np.linalg.norm(reached - wp[name])
        print(name, "error:", err)

        arm.set_joint_positions(q)

        if gripper_action == 'open':
            arm.open_gripper()
            arm.wait(1.5)
        elif gripper_action == 'close':
            arm.close_gripper()
            arm.wait(1.5)

    arm.move_to(HOME)




# ============================================================
# MAIN — DO NOT MODIFY
# ============================================================
def main():
    # raise NotImplementedError("TODO: Implement main() to run the pick-and-place demo.")
    # forward_kinematics([0, 0, 0, 0, 0, 0])
    # np.set_printoptions(precision=4, suppress=True)
    # print(compute_jacobian([0, 0, 0, 0, 0, 0]))
    global ROBOT_POS

    robot = Supervisor()
    arm = UR5eInterface(robot)

        # TEMP: list everything in the world so we can find the DEF names
    root_children = robot.getRoot().getField("children")
    for i in range(root_children.getCount()):
        node = root_children.getMFNode(i)
        print("DEF:", repr(node.getDef()), " type:", node.getTypeName())

    robot_node = robot.getSelf()
    block_node = robot.getFromDef("BLOCK")
    bin_node = robot.getFromDef("BIN")

    ROBOT_POS = np.array(robot_node.getPosition())
    block_world = np.array(block_node.getPosition())
    bin_world = np.array(bin_node.getPosition())

    pick_and_place(arm, block_world, bin_world)

if __name__ == "__main__":
    main()
