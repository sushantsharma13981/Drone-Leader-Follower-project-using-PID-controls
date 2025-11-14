import pybullet as p
import pybullet_data
import time
import numpy as np
import math

# Import local classes
from DSLPIDControl import DSLPIDControl
from enums import DroneModel

# --- Simulation Constants ---
SIM_HZ = 240.
CTRL_HZ = 48.
CTRL_TIMESTEP = 1.0 / CTRL_HZ
SIM_STEPS = int(SIM_HZ / CTRL_HZ)

# --- Simulation Setup ---
def setup_simulation(drone_urdf_path):
    print("Connecting to PyBullet...")
    if p.isConnected():
        p.disconnect()
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Set up the environment
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(fixedTimeStep=1.0/SIM_HZ)
    
    # Load floor
    p.loadURDF("plane.urdf")
    
    print(f"Loading drones from {drone_urdf_path}...")
    
    # --- DRONE 1 (Leader) ---
    start_pos_1 = [0, 0, 0.5]
    start_orn_1 = p.getQuaternionFromEuler([0, 0, 0])
    droneId1 = p.loadURDF(drone_urdf_path, start_pos_1, start_orn_1)
    
    # --- DRONE 2 (Follower) ---
    # Start 1 meter behind the leader
    start_pos_2 = [-1.0, 0, 0.5]
    start_orn_2 = p.getQuaternionFromEuler([0, 0, 0])
    droneId2 = p.loadURDF(drone_urdf_path, start_pos_2, start_orn_2)
    
    # Make the follower a slightly different color (Reddish) to distinguish it
    p.changeVisualShape(droneId2, -1, rgbaColor=[1, 0.5, 0.5, 1])
    
    return droneId1, droneId2

# --- Main Simulation Loop ---
def run_manual_control(drone1, drone2):
    
    # --- Parameters ---
    MOVE_SPEED = 1.5      # Max speed in m/s
    CLIMB_SPEED = 1.0     # Climb speed in m/s
    FOLLOW_OFFSET = np.array([-1.0, 0.0, 0.0]) # Follower stays 1m behind in X
    
    # --- Instantiate Controllers (One per drone) ---
    ctrl1 = DSLPIDControl(drone_model=DroneModel.CF2P)
    ctrl2 = DSLPIDControl(drone_model=DroneModel.CF2P)
    
    # --- Trajectory State (Leader) ---
    target_pos_1 = np.array([0.0, 0.0, 0.5]) 
    
    print("------------------------------------------------")
    print("MANUAL CONTROL MODE STARTED")
    print("Controls (Leader Drone):")
    print("  [R] Forward   [F] Backward")
    print("  [D] Left      [G] Right")
    print("  [UP] Ascend   [DOWN] Descend")
    print("------------------------------------------------")

    while True:
        # 1. Get Keyboard Input
        keys = p.getKeyboardEvents()
        vel_cmd = np.zeros(3) 

        # Forward (R) / Backward (F) -> X axis
        if ord('r') in keys and keys[ord('r')] & p.KEY_IS_DOWN:
            vel_cmd[0] = 1.0
        if ord('f') in keys and keys[ord('f')] & p.KEY_IS_DOWN:
            vel_cmd[0] = -1.0
            
        # Left (D) / Right (G) -> Y axis
        if ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
            vel_cmd[1] = 1.0 
        if ord('g') in keys and keys[ord('g')] & p.KEY_IS_DOWN:
            vel_cmd[1] = -1.0

        # Up / Down (Z axis) -> Arrow Keys
        if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
            vel_cmd[2] = 1.0
        if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
            vel_cmd[2] = -1.0

        # Normalize velocity
        if np.linalg.norm(vel_cmd) > 0:
            xy_norm = np.linalg.norm(vel_cmd[0:2])
            if xy_norm > 0:
                vel_cmd[0:2] = (vel_cmd[0:2] / xy_norm) * MOVE_SPEED
            vel_cmd[2] *= CLIMB_SPEED

        # 2. Update Leader Target ("Carrot")
        target_pos_1 += vel_cmd * CTRL_TIMESTEP
        if target_pos_1[2] < 0.05: target_pos_1[2] = 0.05

        # 3. Get States for Both Drones
        pos1, quat1 = p.getBasePositionAndOrientation(drone1)
        vel1, ang_vel1 = p.getBaseVelocity(drone1)
        
        pos2, quat2 = p.getBasePositionAndOrientation(drone2)
        vel2, ang_vel2 = p.getBaseVelocity(drone2)
        
        # 4. Calculate Follower Target
        # The follower tries to be at Leader's Actual Position + Offset
        target_pos_2 = np.array(pos1) + FOLLOW_OFFSET
        # Match the leader's height exactly (optional, can also use offset[2])
        target_pos_2[2] = target_pos_1[2] 

        # 5. Compute Control (Leader)
        rpm1, _, _ = ctrl1.computeControl(
            control_timestep=CTRL_TIMESTEP,
            cur_pos=np.array(pos1),
            cur_quat=np.array(quat1),
            cur_vel=np.array(vel1),
            cur_ang_vel=np.array(ang_vel1),
            target_pos=target_pos_1,
            target_vel=vel_cmd
        )
        
        # 6. Compute Control (Follower)
        # Follower tries to match the leader's velocity to stay smooth
        rpm2, _, _ = ctrl2.computeControl(
            control_timestep=CTRL_TIMESTEP,
            cur_pos=np.array(pos2),
            cur_quat=np.array(quat2),
            cur_vel=np.array(vel2),
            cur_ang_vel=np.array(ang_vel2),
            target_pos=target_pos_2,
            target_vel=np.array(vel1) # Match leader's actual velocity
        )
        
        # 7. Physics Step
        forces1 = ctrl1.KF * (rpm1**2)
        forces2 = ctrl2.KF * (rpm2**2)
        
        for _ in range(SIM_STEPS):
            # Apply forces to Leader
            p.applyExternalForce(drone1, 0, [0, 0, forces1[0]], [0, 0, 0], p.LINK_FRAME)
            p.applyExternalForce(drone1, 1, [0, 0, forces1[1]], [0, 0, 0], p.LINK_FRAME)
            p.applyExternalForce(drone1, 2, [0, 0, forces1[2]], [0, 0, 0], p.LINK_FRAME)
            p.applyExternalForce(drone1, 3, [0, 0, forces1[3]], [0, 0, 0], p.LINK_FRAME)
            
            # Apply forces to Follower
            p.applyExternalForce(drone2, 0, [0, 0, forces2[0]], [0, 0, 0], p.LINK_FRAME)
            p.applyExternalForce(drone2, 1, [0, 0, forces2[1]], [0, 0, 0], p.LINK_FRAME)
            p.applyExternalForce(drone2, 2, [0, 0, forces2[2]], [0, 0, 0], p.LINK_FRAME)
            p.applyExternalForce(drone2, 3, [0, 0, forces2[3]], [0, 0, 0], p.LINK_FRAME)
            
            p.stepSimulation()
            time.sleep(1./SIM_HZ)

if __name__ == "__main__":
    d1, d2 = setup_simulation("cf2p.urdf")
    run_manual_control(d1, d2)