#!/usr/bin/env python
import argparse
import rospy
import os
import tf
from autolab_core import RigidTransform
import numpy as np

def main(args):
    STUD_WIDTH = 0.008
    # Directory containing the .tf files
    transforms_dir = "/home/mfi/repos/ros1_ws/src/kevin/vibro_tactile_toolbox/transforms/T_lego_world"

    # List to hold the transforms
    transforms = []
    p0 = None
    # Get a list of all files in the directory
    files = os.listdir(transforms_dir)

    # Filter only .tf files
    tf_files = [f for f in files if f.endswith('.tf')]
    for tf_file in tf_files:
        # Construct the full path to the .tf file
        tf_path = os.path.join(transforms_dir, tf_file)
        transform = RigidTransform.load(tf_path)

        tf_file = tf_file.replace(".", "_")
        _, _, sx, sy, _ = tf_file.split("_")
        sx, sy = int(sx), int(sy)
        p = transform.translation[:, None]
        if (sx == 0 and sy == 0):
            p0 = p

        transforms.append((sx, sy, p))

    # Construct correlation matrix H
    # L = SUM(R@S - DELTA_P)
    # S = [s0, s1, ...]
    # DELTA_p = [pi1j1 - p00, pi2j2 - p00, ...]
    s_vecs = []
    dp_vecs = []
    for sx, sy, p in transforms:
        s = np.array([[sx], [sy], [0]])
        s_vecs.append(s)

        dp = p - p0
        dp_vecs.append(dp)

    S = np.hstack(s_vecs)
    P = np.hstack(dp_vecs) / STUD_WIDTH
    H = P @ S.T

    # print(S)
    # print(P)
    # print(H)

    # Perform SVD
    U, S, Vh = np.linalg.svd(H)

    # Find optimal rotation matrix
    R_opt = U @ np.diag([1, 1, -1]) @ Vh
    print(f"Optimal R: ")
    for i in range(3):
        for j in range(3):
            print(R_opt[i, j], end="\t")
        print()
    print()
    print(f"Det(R_opt) = {np.linalg.det(R_opt)}")

    if not args.overwrite:
        return
    
    # overwrite the new rotation matrix to all
    for tf_file in tf_files:
        tf_path = os.path.join(transforms_dir, tf_file)
        transform = RigidTransform.load(tf_path)
        transform.rotation = R_opt
        RigidTransform.save(transform, tf_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--overwrite', '-o', action='store_true',
                        help='Use to overwrite all transforms with the optimized rotation matrix. Default is False')
    args = parser.parse_args()
    main(args)
