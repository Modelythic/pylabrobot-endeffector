#!/usr/bin/env python3
"""
Animate substrate transfer with robot arm motion and gripper control
"""

import os
import sys
import time
import math
import signal

# Fix OpenGL issues before importing pybullet
os.environ['MESA_GL_VERSION_OVERRIDE'] = '3.3'
os.environ['MESA_GLSL_VERSION_OVERRIDE'] = '330'

try:
    import pybullet as p
    import pybullet_data
    import numpy as np
    from PIL import Image
except ImportError as e:
    print(f"Error: Missing required package: {e}")
    print("Install with: pip install pybullet pillow numpy")
    sys.exit(1)

try:
    import imageio
    HAS_IMAGEIO = True
except ImportError:
    print("Warning: imageio not installed. Will save frames as images only.")
    HAS_IMAGEIO = False


def main():
    # Shared state for signal handler
    frames = []
    frame_count = [0]  # Use list to allow modification in nested function
    output_file = "substrate_transfer_animation.mp4"
    frames_dir = "animation_frames"
    interrupted = [False]  # Track if we were interrupted

    def save_video():
        """Save the video with frames captured so far"""
        if HAS_IMAGEIO and len(frames) > 0:
            print(f"\n\nSaving video with {frame_count[0]} frames...")
            imageio.mimsave(output_file, frames, fps=30, quality=8, macro_block_size=1)
            print(f"✓ Video saved to: {output_file}")
            file_size = os.path.getsize(output_file) / (1024 * 1024)
            print(f"  File size: {file_size:.2f} MB")
            print(f"  Duration: {frame_count[0] / 30:.1f} seconds")
        elif not HAS_IMAGEIO and frame_count[0] > 0:
            print(f"\n✓ Individual frames saved to: {frames_dir}/")
            print(f"  Total frames: {frame_count[0]}")
        else:
            print("\nNo frames to save.")

    def signal_handler(sig, frame):
        """Handle Ctrl+C gracefully"""
        print("\n\n" + "="*60)
        print("INTERRUPTED BY USER (Ctrl+C)")
        print("="*60)
        interrupted[0] = True
        save_video()
        p.disconnect()
        print("\nDisconnected from PyBullet")
        sys.exit(0)

    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    # Use DIRECT mode for video export
    print("Connecting in DIRECT mode (headless rendering)...")
    print("(Press Ctrl+C at any time to save progress and exit)")
    physicsClient = p.connect(p.DIRECT)
    print("✓ Successfully connected in DIRECT mode")

    # Set up PyBullet
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Load ground plane
    planeId = p.loadURDF("plane.urdf")

    # Load the robot URDF
    urdf_path = "piper_ros/src/piper_description/urdf/piper_description_new.urdf"

    if not os.path.exists(urdf_path):
        print(f"Error: URDF file not found at {urdf_path}")
        p.disconnect()
        sys.exit(1)

    print(f"Loading URDF: {urdf_path}")
    robotId = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
    print(f"✓ Robot loaded successfully with ID: {robotId}")

    # Get number of joints and identify important joints
    numJoints = p.getNumJoints(robotId)
    print(f"✓ Number of joints: {numJoints}")

    # Map joint names to indices
    joint_map = {}
    link_map = {}
    for i in range(numJoints):
        jointInfo = p.getJointInfo(robotId, i)
        jointName = jointInfo[1].decode('utf-8')
        linkName = jointInfo[12].decode('utf-8')
        joint_map[jointName] = i
        link_map[linkName] = i

    # Important joints
    arm_joints = [
        joint_map['joint1'],  # Base rotation
        joint_map['joint2'],  # Shoulder
        joint_map['joint3'],  # Elbow
        joint_map['joint4'],  # Wrist 1
        joint_map['joint5'],  # Wrist 2
        joint_map['joint6']   # Wrist 3
    ]

    gripper_joint = joint_map['finger1_joint']  # Master gripper joint
    palm_link_idx = link_map['palm_link']  # End effector link

    print(f"\nArm joints: {arm_joints}")
    print(f"Gripper joint: {gripper_joint}")
    print(f"Palm link index: {palm_link_idx}")

    def log_pose_delta(label, link_name, reference_pos=None):
        idx = link_map.get(link_name)
        if idx is None:
            print(f"[POSE DEBUG] {label}: link '{link_name}' not found.")
            return
        link_state = p.getLinkState(robotId, idx, computeForwardKinematics=True)
        pos = np.array(link_state[4])
        quat = link_state[5]
        rpy = np.array(p.getEulerFromQuaternion(quat))
        message = (
            f"[POSE DEBUG] {label}: link='{link_name}' "
            f"pos={np.round(pos, 4)} rpy(rad)={np.round(rpy, 4)}"
        )
        if reference_pos is not None:
            delta = pos - np.array(reference_pos, dtype=float)
            message += f" delta_vs_ref={np.round(delta, 4)}"
        print(message)

    # Tray position in world coordinates
    tray_world_pos = [-0.35, 0.2, 0.0]

    # Substrate positions relative to tray (5x5 grid, 15mm pitch)
    substrate_positions = []
    for row in range(5):
        for col in range(5):
            x = (col - 2) * 0.015  # Relative to tray center
            y = (row - 2) * 0.015
            z = 0.0019  # Height above tray base
            substrate_positions.append([
                tray_world_pos[0] + x,
                tray_world_pos[1] + y,
                tray_world_pos[2] + z
            ])

    # Spincoater position
    spincoater_pos = [0.35, 0.25, 0.07]

    log_pose_delta("Tray base (URDF)", "tray_base_link", tray_world_pos)
    log_pose_delta("Tray substrates (URDF)", "tray_substrate_array", tray_world_pos)
    log_pose_delta("Spincoater base (URDF)", "spincoater_base_link", spincoater_pos)
    log_pose_delta("Spincoater chuck (URDF)", "spincoater_chuck", spincoater_pos)

    # Define motion sequence
    def move_to_position(target_pos, target_orn_euler=[0, math.pi, 0], duration=2.0):
        """Move end effector to target position with orientation"""
        target_pos_arr = np.array(target_pos, dtype=float)
        target_rpy_arr = np.array(target_orn_euler, dtype=float)
        target_quat = p.getQuaternionFromEuler(target_orn_euler)
        print(
            f"[IK DEBUG] Commanding move_to_position: "
            f"target_pos={np.round(target_pos_arr, 4)} "
            f"target_rpy={np.round(target_rpy_arr, 4)}"
        )
        steps = int(duration * 240)
        for step in range(steps):
            # Use IK to get joint positions
            jointPoses = p.calculateInverseKinematics(
                robotId,
                palm_link_idx,
                target_pos_arr,
                target_quat,
                maxNumIterations=100,
                residualThreshold=0.001
            )

            # Set arm joint positions
            for i, joint_idx in enumerate(arm_joints):
                p.setJointMotorControl2(
                    robotId,
                    joint_idx,
                    p.POSITION_CONTROL,
                    targetPosition=jointPoses[i],
                    force=100
                )

            p.stepSimulation()

            if step == steps - 1:
                ee_state = p.getLinkState(robotId, palm_link_idx, computeForwardKinematics=True)
                actual_pos = np.array(ee_state[4])
                actual_quat = ee_state[5]
                actual_rpy = np.array(p.getEulerFromQuaternion(actual_quat))
                pos_error = actual_pos - target_pos_arr
                orn_error = actual_rpy - target_rpy_arr
                print(
                    f"[IK DEBUG] target_pos={np.round(target_pos_arr, 4)} "
                    f"actual_pos={np.round(actual_pos, 4)} "
                    f"pos_error={np.round(pos_error, 4)} "
                    f"|pos_error|={np.linalg.norm(pos_error):.5f}"
                )
                print(
                    f"[IK DEBUG] target_rpy={np.round(target_rpy_arr, 4)} "
                    f"actual_rpy={np.round(actual_rpy, 4)} "
                    f"rpy_error={np.round(orn_error, 4)}"
                )

            p.stepSimulation()
            yield step

    def control_gripper(opening, duration=1.0):
        """Control gripper opening (0=closed, 0.03=fully open)"""
        steps = int(duration * 240)
        for step in range(steps):
            p.setJointMotorControl2(
                robotId,
                gripper_joint,
                p.POSITION_CONTROL,
                targetPosition=opening,
                force=100
            )
            p.stepSimulation()
            yield step

    # Set up camera for video rendering
    width, height = 1920, 1080
    fov = 60
    aspect = width / height
    near = 0.02
    far = 5.0

    # Camera position
    camera_target = [0, 0, 0.5]
    camera_distance = 1.8
    camera_yaw = 30
    camera_pitch = -35

    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=camera_target,
        distance=camera_distance,
        yaw=camera_yaw,
        pitch=camera_pitch,
        roll=0,
        upAxisIndex=2
    )

    projection_matrix = p.computeProjectionMatrixFOV(
        fov=fov,
        aspect=aspect,
        nearVal=near,
        farVal=far
    )

    print(f"\nVideo settings: {width}x{height} @ 30 fps")

    # Output video file - already defined at top of main()
    if not os.path.exists(frames_dir):
        os.makedirs(frames_dir)

    print(f"Output file: {output_file}")

    # frames and frame_count already defined at top of main()
    frame_interval = 8  # Capture every 8 steps for 30fps

    def capture_frame():
        """Capture a frame for video"""
        img_arr = p.getCameraImage(
            width, height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=p.ER_TINY_RENDERER
        )
        rgb_array = np.array(img_arr[2], dtype=np.uint8).reshape(height, width, 4)[:, :, :3]

        if HAS_IMAGEIO:
            frames.append(rgb_array)
        else:
            frame_path = os.path.join(frames_dir, f"frame_{frame_count[0]:04d}.png")
            Image.fromarray(rgb_array).save(frame_path)

        frame_count[0] += 1

    print("\n" + "="*60)
    print("EXECUTING SUBSTRATE TRANSFER SEQUENCE")
    print("="*60)

    # Animation sequence
    sim_step = 0

    # Initial position - above home
    print("\n1. Moving to home position...")
    home_pos = [0, 0.3, 0.4]
    for _ in move_to_position(home_pos):
        if sim_step % frame_interval == 0:
            capture_frame()
        sim_step += 1
    print(f"   ✓ Home position reached (step {sim_step})")

    # Open gripper
    print("\n2. Opening gripper...")
    for _ in control_gripper(0.025):
        if sim_step % frame_interval == 0:
            capture_frame()
        sim_step += 1
    print(f"   ✓ Gripper opened (step {sim_step})")

    # Move to first substrate (center of tray)
    substrate_idx = 12  # Center substrate (row 2, col 2)
    substrate_pos = substrate_positions[substrate_idx].copy()
    approach_pos = substrate_pos.copy()
    approach_pos[2] += 0.1  # 10cm above substrate

    print(f"\n3. Moving to substrate #{substrate_idx} approach...")
    for _ in move_to_position(approach_pos):
        if sim_step % frame_interval == 0:
            capture_frame()
        sim_step += 1
    print(f"   ✓ Approach position reached (step {sim_step})")

    # Lower to substrate
    print("\n4. Lowering to substrate...")
    pickup_pos = substrate_pos.copy()
    pickup_pos[2] += 0.005  # Just above substrate
    for _ in move_to_position(pickup_pos, duration=1.5):
        if sim_step % frame_interval == 0:
            capture_frame()
        sim_step += 1
    print(f"   ✓ Pickup position reached (step {sim_step})")

    # Close gripper
    print("\n5. Closing gripper to grasp substrate...")
    for _ in control_gripper(0.008, duration=1.0):  # Close to 8mm (substrate is 10mm)
        if sim_step % frame_interval == 0:
            capture_frame()
        sim_step += 1
    print(f"   ✓ Gripper closed (step {sim_step})")

    # Lift substrate
    print("\n6. Lifting substrate...")
    lift_pos = pickup_pos.copy()
    lift_pos[2] += 0.15
    for _ in move_to_position(lift_pos):
        if sim_step % frame_interval == 0:
            capture_frame()
        sim_step += 1
    print(f"   ✓ Substrate lifted (step {sim_step})")

    # Move to spincoater
    print("\n7. Moving to spincoater...")
    spincoater_approach = spincoater_pos.copy()
    spincoater_approach[2] += 0.1
    for _ in move_to_position(spincoater_approach):
        if sim_step % frame_interval == 0:
            capture_frame()
        sim_step += 1
    print(f"   ✓ Spincoater approach reached (step {sim_step})")

    # Lower to spincoater
    print("\n8. Placing substrate on spincoater...")
    for _ in move_to_position(spincoater_pos, duration=1.5):
        if sim_step % frame_interval == 0:
            capture_frame()
        sim_step += 1
    print(f"   ✓ Placement position reached (step {sim_step})")

    # Open gripper
    print("\n9. Releasing substrate...")
    for _ in control_gripper(0.025):
        if sim_step % frame_interval == 0:
            capture_frame()
        sim_step += 1
    print(f"   ✓ Substrate released (step {sim_step})")

    # Retract
    print("\n10. Retracting...")
    retract_pos = spincoater_pos.copy()
    retract_pos[2] += 0.15
    for _ in move_to_position(retract_pos):
        if sim_step % frame_interval == 0:
            capture_frame()
        sim_step += 1
    print(f"   ✓ Retracted (step {sim_step})")

    # Return home
    print("\n11. Returning to home...")
    for _ in move_to_position(home_pos):
        if sim_step % frame_interval == 0:
            capture_frame()
        sim_step += 1
    print(f"   ✓ Home position reached (step {sim_step})")

    print("\n" + "="*60)
    print(f"SEQUENCE COMPLETE - Total steps: {sim_step}")
    print(f"Total frames captured: {frame_count[0]}")
    print("="*60)

    # Save video (only if not interrupted, otherwise already saved)
    if not interrupted[0]:
        save_video()

    # Disconnect
    p.disconnect()
    print("\nDisconnected from PyBullet")


if __name__ == "__main__":
    main()
