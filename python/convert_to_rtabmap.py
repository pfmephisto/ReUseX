# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
import os
import numpy as np
from ruamel.yaml import YAML
from ruamel.yaml.comments import CommentedSeq
import cv2
#from skvideo import io
from PIL import Image
#from stray_visualize import DEPTH_WIDTH, DEPTH_HEIGHT, _resize_camera_matrix
import pandas as pd
from scipy.spatial.transform import Rotation as R

FRAME_WIDTH = 1920
FRAME_HEIGHT = 1440
OUT_WIDTH = 640
OUT_HEIGHT = 480

def read_args():
    """Parse command-line arguments for dataset conversion.
    
    Returns:
        Parsed command-line arguments including dataset path, output path, and confidence threshold.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', type=str)
    parser.add_argument('--out', type=str)
    parser.add_argument('--confidence', type=int, default=2)
    return parser.parse_args()

def _resize_camera_matrix(camera_matrix, scale_x, scale_y):
    """Resize camera intrinsic matrix by scaling factors.
    
    Args:
        camera_matrix: 3x3 camera intrinsic matrix.
        scale_x: Horizontal scaling factor.
        scale_y: Vertical scaling factor.
        
    Returns:
        Scaled 3x3 camera intrinsic matrix.
    """
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    return np.array([[fx * scale_x, 0.0, cx * scale_x],
        [0., fy * scale_y, cy * scale_y],
        [0., 0., 1.0]])

def write_frames(flags, rgb_out_dir):
    """Extract and write RGB frames from video to JPEG files.
    
    Args:
        flags: Command-line arguments containing dataset path.
        rgb_out_dir: Output directory for RGB frames.
    """
    rgb_video = os.path.join(flags.dataset, 'rgb.mp4')

    # Open video file using OpenCV
    cap = cv2.VideoCapture(rgb_video)
    if not cap.isOpened():
        raise IOError(f"Cannot open video file: {rgb_video}")

    os.makedirs(rgb_out_dir, exist_ok=True)

    frame_idx = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break  # End of video
        
        print(f"Writing rgb frame {frame_idx:06}" + " " * 10, end='\r')

        # Resize and save as JPEG
        frame_resized = cv2.resize(frame, (OUT_WIDTH, OUT_HEIGHT))
        frame_path = os.path.join(rgb_out_dir, f"{frame_idx:06}.jpg")
        params = [cv2.IMWRITE_JPEG_QUALITY, 90]
        cv2.imwrite(frame_path, frame_resized, params)

        frame_idx += 1

    cap.release()
    print(f"\nFinished writing {frame_idx} frames to {rgb_out_dir}")

# def write_frames(flags, rgb_out_dir):
#     rgb_video = os.path.join(flags.dataset, 'rgb.mp4')
#     video = io.vreader(rgb_video)
#     for i, frame in enumerate(video):
#         print(f"Writing rgb frame {i:06}" + " " * 10, end='\r')
#         frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
#         frame = cv2.resize(frame, (OUT_WIDTH, OUT_HEIGHT))
#         frame_path = os.path.join(rgb_out_dir, f"{i:06}.jpg")
#         params = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
#         cv2.imwrite(frame_path, frame, params)

def resize_depth(depth):
    """Resize depth image with nearest neighbor interpolation.
    
    Args:
        depth: Input depth image array.
        
    Returns:
        Resized depth image with values below 10 set to 0.
    """
    out = cv2.resize(depth, (OUT_WIDTH, OUT_HEIGHT), interpolation=cv2.INTER_NEAREST_EXACT)
    out[out < 10] = 0
    return out

def write_depth(flags, depth_out_dir):
    """Process and write depth frames with confidence filtering.
    
    Args:
        flags: Command-line arguments containing dataset path and confidence threshold.
        depth_out_dir: Output directory for depth frames.
    """
    depth_dir_in = os.path.join(flags.dataset, 'depth')
    confidence_dir = os.path.join(flags.dataset, 'confidence')
    files = sorted(os.listdir(depth_dir_in))
    for index, filename in enumerate(files):
        if not ('.npy' in filename or '.png' in filename):
            continue
        if index == len(files)-1:
            # ignore last file to match the color number
            break
        print(f"Writing depth frame {filename}", end='\r')
        number, _ = filename.split('.')
        confidence = cv2.imread(os.path.join(confidence_dir, number + '.png'))[:, :, 0]
        depth = np.array(Image.open(os.path.join(depth_dir_in, filename)))
        depth[confidence < flags.confidence] = 0
        depth = resize_depth(depth)
        cv2.imwrite(os.path.join(depth_out_dir, f'{number}.png'), depth)

def write_intrinsics(flags):
    """Write camera intrinsics to YAML file.
    
    Scales the camera intrinsics from original resolution to output resolution.
    
    Args:
        flags: Command-line arguments containing dataset and output paths.
    """
    yaml = YAML()
    yaml.width = 60
    intrinsics = np.loadtxt(os.path.join(flags.dataset, 'camera_matrix.csv'), delimiter=',')
    data = {}
    intrinsics_scaled = _resize_camera_matrix(intrinsics, OUT_WIDTH / FRAME_WIDTH, OUT_HEIGHT / FRAME_HEIGHT)
    data['camera_name'] = 'Stray Scanner dataset'
    data['image_width'] = OUT_WIDTH
    data['image_height'] = OUT_HEIGHT
    camera_matrix = {}
    camera_matrix['rows'] = 3
    camera_matrix['cols'] = 3
    camera_matrix['data'] = CommentedSeq(
           [float(intrinsics_scaled[0, 0]), 0.0, float(intrinsics_scaled[0, 2]),
            0.0, float(intrinsics_scaled[1, 1]), float(intrinsics_scaled[1, 2]),
            0.0, 0.0, 1.0])
    data['camera_matrix'] = camera_matrix
    
    data['camera_matrix']['data'].fa.set_flow_style()
    with open(os.path.join(flags.out, 'camera_intrinsics.yaml'), 'wt') as f:
        f.write("%YAML:1.0\n---\n") # compatibility with opencv's yaml loader
        yaml.dump(data, f)

def write_odometry(flags):
    """Convert and write odometry data from OpenGL to ROS coordinate frame.
    
    Reorganizes odometry from: timestamp frame x y z qx qy qz qw
    to: timestamp x y z qx qy qz qw frame
    
    Args:
        flags: Command-line arguments containing dataset and output paths.
    """
    # Reorganize as below from while converting poses from opengl to ROS coordinate frame.
    # From:
    #  timestamp frame x y z qx qy qz qw
    # to:
    #  timestamp x y z qx qy qz qw frame
    
    df = pd.read_csv(os.path.join(flags.dataset, 'odometry.csv'), sep=r'[\s,]+', engine='python')

    # Extract position and orientation
    positions = df[['x', 'y', 'z']].values
    orientations = df[['qx', 'qy', 'qz', 'qw']].values

    Q = R.from_quat([ 0.5, -0.5, -0.5, 0.5 ])
    Q_inv = Q.inv()
    
    # This rotation has been added to original pose, remove it (https://github.com/strayrobots/scanner/blob/e0e252221048b80d288dc1f264df775aacbe4304/StrayScanner/Helpers/OdometryEncoder.swift#L37)
    q_AC = R.from_quat([ 1., 0., 0., 0. ])
    q_AC_inv = q_AC.inv()

    # Apply Q * position * Q^-1 (rotate each position by Q)
    rotated_positions = Q.apply(positions)

    # Apply Q * orientation * Q^-1 (rotate each orientation)
    rotated_orientations = []
    for q in orientations:
        rot = R.from_quat(q) * q_AC_inv
        new_rot = Q * rot * Q_inv
        rotated_orientations.append(new_rot.as_quat())

    rotated_orientations = np.array(rotated_orientations)

    df_out = df.copy()
    df_out[['x', 'y', 'z']] = rotated_positions
    df_out[['qx', 'qy', 'qz', 'qw']] = rotated_orientations
    
    # move "id" column at the end
    cols = [col for col in df_out.columns if col != 'frame'] + ['frame']
    df_out = df_out[cols]

    # ignore last entry to match the color number
    df_out = df_out[:-1] 

    # Save to file
    df_out.to_csv(os.path.join(flags.out, 'odometry.txt'), sep=' ', index=False, float_format='%.8f')
    df_out['timestamp'].to_csv(os.path.join(flags.out, 'timestamps.txt'), header=False, index=False, float_format='%.8f')
    
def write_imu(flags):
    """Convert IMU data to EuRoC dataset format.
    
    Converts from: timestamp(float), a_x, a_y, a_z, alpha_x, alpha_y, alpha_z
    to: timestamp [ns], w_RS_S_x [rad s^-1], w_RS_S_y [rad s^-1], w_RS_S_z [rad s^-1],
        a_RS_S_x [m s^-2], a_RS_S_y [m s^-2], a_RS_S_z [m s^-2]
    
    Args:
        flags: Command-line arguments containing dataset and output paths.
    """
    # Convert imu data to EuRoC dataset format:
    # From:
    #  timestamp(float), a_x, a_y, a_z, alpha_x, alpha_y, alpha_z
    # to:
    #  timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
    
    df = pd.read_csv(os.path.join(flags.dataset, 'imu.csv'), sep=r'[\s,]+', engine='python')

    # Extract position and orientation
    timestamps = df['timestamp'].to_numpy()
    
    timestamps = timestamps*(10**9)
    
    df_out = df.copy()
    df_out['timestamp'] = np.array(timestamps).astype(np.int64)
    df_out['timestamp'] = df_out['timestamp'].astype(str).apply(lambda x: x.zfill(18))
    
    # Transform gyro (x->up, y->left, z->backward) in base frame (x->forward, y->left, z->up)
    df_out['alpha_x'] = df['alpha_z'].to_numpy() * -1
    df_out['alpha_z'] = df['alpha_x'].to_numpy()
    
    # Transform acc (x->down, y->right, z->forward) in base frame (x->forward, y->left, z->up)
    df_out['a_x'] = df['a_z'].to_numpy()
    df_out['a_y'] = df['a_y'].to_numpy() * -1
    df_out['a_z'] = df['a_x'].to_numpy() * -1

    # Save to file
    df_out = df_out[['timestamp', 'alpha_x', 'alpha_y', 'alpha_z', 'a_x', 'a_y', 'a_z']]
    df_out.to_csv(os.path.join(flags.out, 'imu.csv'), sep=',', index=False, float_format='%.16f')
    

def main():
    """Main function to convert Stray Scanner dataset to RTABMap format.
    
    Processes RGB frames, depth images, camera intrinsics, odometry, and IMU data.
    """
    flags = read_args()
    rgb_out = os.path.join(flags.out, 'color/')
    depth_out = os.path.join(flags.out, 'depth/')
    os.makedirs(rgb_out, exist_ok=True)
    os.makedirs(depth_out, exist_ok=True)

    write_intrinsics(flags)
    write_odometry(flags)
    write_imu(flags)
    write_depth(flags, depth_out)
    write_frames(flags, rgb_out)
    print("\nDone.")

if __name__ == "__main__":
    main()
