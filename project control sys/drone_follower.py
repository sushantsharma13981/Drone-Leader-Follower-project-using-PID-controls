import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import os


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

    # --- LOGGING ARRAYS ---
    log_time = []
    log_leader_rpy = []
    log_follower_rpy = []
    log_leader_pos = []
    log_follower_pos = []
    log_leader_vel = []
    log_follower_vel = []

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

    try:
        pass
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

            # Convert quaternion → Euler angles
            rpy1 = np.array(p.getEulerFromQuaternion(quat1))
            rpy2 = np.array(p.getEulerFromQuaternion(quat2))

            log_time.append(time.time())
            log_leader_rpy.append(rpy1)
            log_follower_rpy.append(rpy2)

            log_leader_pos.append(np.array(pos1))
            log_follower_pos.append(np.array(pos2))

            log_leader_vel.append(np.array(vel1))
            log_follower_vel.append(np.array(vel2))

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


    except KeyboardInterrupt:
        print("\nSimulation stopped. Saving all graphs into /graphs folder...")

        # Create folder
        save_dir = "graphs"
        os.makedirs(save_dir, exist_ok=True)

        # Convert logs
        time_arr = np.array(log_time)
        leader_rpy = np.array(log_leader_rpy)
        follower_rpy = np.array(log_follower_rpy)

        leader_pos = np.array(log_leader_pos)
        follower_pos = np.array(log_follower_pos)

        leader_vel = np.array(log_leader_vel)
        follower_vel = np.array(log_follower_vel)

        # ---------- 1. RPY ----------
        labels = ["Roll", "Pitch", "Yaw"]
        plt.figure(figsize=(12, 8))
        for i in range(3):
            plt.subplot(3, 1, i + 1)
            plt.plot(time_arr, leader_rpy[:, i], label="Leader")
            plt.plot(time_arr, follower_rpy[:, i], label="Follower")
            plt.ylabel(labels[i])
            plt.grid()
            plt.legend()
        plt.xlabel("Time (s)")
        plt.suptitle("Leader vs Follower - RPY")
        plt.tight_layout()
        plt.savefig(f"{save_dir}/leader_follower_rpy.png", dpi=300)
        plt.close()

        # ---------- 2. XYZ ----------
        plt.figure(figsize=(12, 8))
        labels = ["X", "Y", "Z"]
        for i in range(3):
            plt.subplot(3, 1, i + 1)
            plt.plot(time_arr, leader_pos[:, i], label="Leader")
            plt.plot(time_arr, follower_pos[:, i], label="Follower")
            plt.ylabel(labels[i])
            plt.grid()
            plt.legend()
        plt.xlabel("Time (s)")
        plt.suptitle("Leader vs Follower - Position XYZ")
        plt.tight_layout()
        plt.savefig(f"{save_dir}/leader_follower_xyz.png", dpi=300)
        plt.close()

        # ---------- 3. Velocities ----------
        plt.figure(figsize=(12, 8))
        labels = ["Vx", "Vy", "Vz"]
        for i in range(3):
            plt.subplot(3, 1, i + 1)
            plt.plot(time_arr, leader_vel[:, i], label="Leader")
            plt.plot(time_arr, follower_vel[:, i], label="Follower")
            plt.ylabel(labels[i])
            plt.grid()
            plt.legend()
        plt.xlabel("Time (s)")
        plt.suptitle("Leader vs Follower - Velocities")
        plt.tight_layout()
        plt.savefig(f"{save_dir}/leader_follower_velocity.png", dpi=300)
        plt.close()

        # ---------- 4. Attitude Error ----------
        rpy_err = leader_rpy - follower_rpy
        labels = ["Roll Error", "Pitch Error", "Yaw Error"]

        plt.figure(figsize=(12, 8))
        for i in range(3):
            plt.subplot(3, 1, i + 1)
            plt.plot(time_arr, rpy_err[:, i])
            plt.ylabel(labels[i])
            plt.grid()
        plt.xlabel("Time (s)")
        plt.suptitle("Attitude Error (Leader - Follower)")
        plt.tight_layout()
        plt.savefig(f"{save_dir}/attitude_error_rpy.png", dpi=300)
        plt.close()

        # ---------- 5. 3D Trajectory ----------
        from mpl_toolkits.mplot3d import Axes3D
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        ax.plot(leader_pos[:, 0], leader_pos[:, 1], leader_pos[:, 2], label="Leader")
        ax.plot(follower_pos[:, 0], follower_pos[:, 1], follower_pos[:, 2], label="Follower")

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.legend()
        ax.set_title("3D Trajectory")

        plt.tight_layout()
        plt.savefig(f"{save_dir}/trajectory_3d.png", dpi=300)
        plt.close()

        print("\nAll graphs saved successfully inside the 'graphs' folder!")
        p.disconnect()

        # -------------------------------------------------------
        # 6. COMBINED 2×2 RESEARCH-STYLE FIGURE
        # -------------------------------------------------------

        fig, axs = plt.subplots(2, 2, figsize=(15, 10))

        # ---------- Subplot 1: Yaw-only or RPY ----------
        axs[0, 0].plot(time_arr, leader_rpy[:, 2], label="Leader Yaw")
        axs[0, 0].plot(time_arr, follower_rpy[:, 2], label="Follower Yaw")
        axs[0, 0].set_title("Yaw Tracking")
        axs[0, 0].set_xlabel("Time (s)")
        axs[0, 0].set_ylabel("Yaw (rad)")
        axs[0, 0].grid()
        axs[0, 0].legend()

        # ---------- Subplot 2: XYZ Position ----------
        axs[0, 1].plot(time_arr, leader_pos[:, 0], label="Leader X")
        axs[0, 1].plot(time_arr, leader_pos[:, 1], label="Leader Y")
        axs[0, 1].plot(time_arr, leader_pos[:, 2], label="Leader Z")
        axs[0, 1].plot(time_arr, follower_pos[:, 0], '--', label="Follower X")
        axs[0, 1].plot(time_arr, follower_pos[:, 1], '--', label="Follower Y")
        axs[0, 1].plot(time_arr, follower_pos[:, 2], '--', label="Follower Z")
        axs[0, 1].set_title("Position Tracking (XYZ)")
        axs[0, 1].set_xlabel("Time (s)")
        axs[0, 1].set_ylabel("Position (m)")
        axs[0, 1].grid()
        axs[0, 1].legend()

        # ---------- Subplot 3: Velocity ----------
        axs[1, 0].plot(time_arr, leader_vel[:, 0], label="Leader Vx")
        axs[1, 0].plot(time_arr, leader_vel[:, 1], label="Leader Vy")
        axs[1, 0].plot(time_arr, leader_vel[:, 2], label="Leader Vz")
        axs[1, 0].plot(time_arr, follower_vel[:, 0], '--', label="Follower Vx")
        axs[1, 0].plot(time_arr, follower_vel[:, 1], '--', label="Follower Vy")
        axs[1, 0].plot(time_arr, follower_vel[:, 2], '--', label="Follower Vz")
        axs[1, 0].set_title("Velocity Tracking (Vx, Vy, Vz)")
        axs[1, 0].set_xlabel("Time (s)")
        axs[1, 0].set_ylabel("Velocity (m/s)")
        axs[1, 0].grid()
        axs[1, 0].legend()

        # ---------- Subplot 4: Attitude Error ----------
        axs[1, 1].plot(time_arr, rpy_err[:, 0], label="Roll Error")
        axs[1, 1].plot(time_arr, rpy_err[:, 1], label="Pitch Error")
        axs[1, 1].plot(time_arr, rpy_err[:, 2], label="Yaw Error")
        axs[1, 1].set_title("Attitude Error (Leader - Follower)")
        axs[1, 1].set_xlabel("Time (s)")
        axs[1, 1].set_ylabel("Error (rad)")
        axs[1, 1].grid()
        axs[1, 1].legend()

        plt.tight_layout()
        plt.savefig(f"{save_dir}/combined_2x2_results.png", dpi=300)
        plt.close()

        print(" - combined_2x2_results.png")

        # -------------------------------------------------------
        # 7. COMBINED GROUP OF 3 (RPY, XYZ, VELOCITY)
        # -------------------------------------------------------
        fig, axs = plt.subplots(3, 1, figsize=(14, 12))

        # ---------- Group 1: RPY Tracking ----------
        axs[0].plot(time_arr, leader_rpy[:, 0], label="Leader Roll")
        axs[0].plot(time_arr, leader_rpy[:, 1], label="Leader Pitch")
        axs[0].plot(time_arr, leader_rpy[:, 2], label="Leader Yaw")

        axs[0].plot(time_arr, follower_rpy[:, 0], '--', label="Follower Roll")
        axs[0].plot(time_arr, follower_rpy[:, 1], '--', label="Follower Pitch")
        axs[0].plot(time_arr, follower_rpy[:, 2], '--', label="Follower Yaw")

        axs[0].set_title("RPY Tracking (Roll, Pitch, Yaw)")
        axs[0].set_xlabel("Time (s)")
        axs[0].set_ylabel("Angle (rad)")
        axs[0].grid()
        axs[0].legend()

        # ---------- Group 2: XYZ Positions ----------
        axs[1].plot(time_arr, leader_pos[:, 0], label="Leader X")
        axs[1].plot(time_arr, leader_pos[:, 1], label="Leader Y")
        axs[1].plot(time_arr, leader_pos[:, 2], label="Leader Z")

        axs[1].plot(time_arr, follower_pos[:, 0], '--', label="Follower X")
        axs[1].plot(time_arr, follower_pos[:, 1], '--', label="Follower Y")
        axs[1].plot(time_arr, follower_pos[:, 2], '--', label="Follower Z")

        axs[1].set_title("XYZ Position Tracking")
        axs[1].set_xlabel("Time (s)")
        axs[1].set_ylabel("Position (m)")
        axs[1].grid()
        axs[1].legend()

        # ---------- Group 3: Velocities ----------
        axs[2].plot(time_arr, leader_vel[:, 0], label="Leader Vx")
        axs[2].plot(time_arr, leader_vel[:, 1], label="Leader Vy")
        axs[2].plot(time_arr, leader_vel[:, 2], label="Leader Vz")

        axs[2].plot(time_arr, follower_vel[:, 0], '--', label="Follower Vx")
        axs[2].plot(time_arr, follower_vel[:, 1], '--', label="Follower Vy")
        axs[2].plot(time_arr, follower_vel[:, 2], '--', label="Follower Vz")

        axs[2].set_title("Velocity Tracking (Vx, Vy, Vz)")
        axs[2].set_xlabel("Time (s)")
        axs[2].set_ylabel("Velocity (m/s)")
        axs[2].grid()
        axs[2].legend()

        plt.tight_layout()
        plt.savefig(f"{save_dir}/combined_grouped_3.png", dpi=300)
        plt.close()

        print(" - combined_grouped_3.png")


if __name__ == "__main__":
    d1, d2 = setup_simulation("cf2p.urdf")
    run_manual_control(d1, d2)