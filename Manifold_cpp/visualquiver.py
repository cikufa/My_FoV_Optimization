#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import subprocess
import argparse
from shutil import copyfile

import numpy as np
from colorama import init, Fore
import matplotlib.pyplot as plt
from matplotlib import rc
from mpl_toolkits.mplot3d import Axes3D

import matplotlib
print(matplotlib.__version__)

_save_ext = ".png"

rc('font', **{'family': 'serif', 'serif': ['Cardo'], 'size': 10})
rc('text', usetex=True)

def fov_plot(points, opt_quivers, BF_quivers, onepointlog, accfile):    
    fig, axes = plt.subplots(1, 4, figsize=(15, 5))
    opt_step = onepointlog[:, 0]
    unique_ids = np.unique(opt_step)

    feature_cnt = onepointlog[:, 7]
    for id_ in unique_ids:
        mask = opt_step == id_
        axes[3].plot(np.where(mask)[0], feature_cnt[mask], marker="s", label=f"ID {int(id_)}")
    axes[3].set_title("feature count")
    axes[3].set_xlabel("step")
    axes[3].set_ylabel("feature count")

    degree_between = onepointlog[:, 8]
    for id_ in unique_ids:
        mask = opt_step == id_
        axes[0].plot(np.where(mask)[0], degree_between[mask], marker="o", label=f"ID {int(id_)}")
    
    axes[0].set_title("Degree Difference Trend")
    axes[0].set_xlabel("step")
    axes[0].set_ylabel("Degree Between")
    
    J_norm = onepointlog[:, 9]
    for id_ in unique_ids:
        mask = opt_step == id_
        axes[1].plot(np.where(mask)[0], J_norm[mask], marker="s", label=f"ID {int(id_)}")
    axes[1].set_title("visibility jacobian norm Trend")
    axes[1].set_xlabel("step")
    axes[1].set_ylabel("aJl norm")

    # step_size = onepointlog[:, 10]
    # for id_ in unique_ids:
    #     mask = opt_step == id_
    #     axes[2].plot(np.where(mask)[0], step_size[mask], marker="s", label=f"ID {int(id_)}")
    # axes[2].set_title("step size")
    # axes[2].set_xlabel("step")
    # axes[2].set_ylabel("step size")


    vis = onepointlog[:, 11]
    vis_bf= accfile[:,1]

    for id_ in unique_ids:
        mask = opt_step == id_
        axes[2].plot(np.where(mask)[0], vis[mask], marker="s", label=f"ID {int(id_)}")
        last_index = np.where(mask)[0][-1]  # Get the last step index for this ID
        axes[2].plot(np.where(mask)[0], np.full(np.sum(mask), vis_bf[int(id_)], dtype=float), color="black", label=f"Max Vis ID {id_}")

    axes[2].set_title("visibility")
    axes[2].set_xlabel("step")
    axes[2].set_ylabel("visibility")

    # plt.show()

# ----------------------------------------------
    # scale_factoropt = 50
    # scale_factor = 30
    # scale_factorbff = 70
    # scale_factorbfv = 60
    # arrow = 0.3

    #monte carlo fif
    scale_factoropt = 0.5
    scale_factor = 0.3
    scale_factorbff = 0.4
    scale_factorbfv = 0.4
    arrow = 0.5

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='blue', alpha=0.05, label='Map Points')
    # ax.scatter(mean[:, 0], mean[:, 1], mean[:, 2], c='blue', s=20, label='Mean ME')
    # ax.scatter(mean[:, 3], mean[:, 4], mean[:, 5], c='black', s=10, label='Mean CHEN')

    points_a = opt_quivers[:, :3]
    points_b = opt_quivers[:, 3:]
    for a, b in zip(points_a, points_b):
        ax.quiver(a[0], a[1], a[2], b[0] * scale_factoropt, b[1] * scale_factoropt, b[2] * scale_factoropt,
                  color='orange', arrow_length_ratio=arrow)

    twc = BF_quivers[:, :3]
    bf_feat = BF_quivers[:, 3:6]
    bf_vis= BF_quivers[:, 6:]
    # for a, b in zip(twc, bf_feat):
    #     ax.quiver(a[0], a[1], a[2], b[0] * scale_factorbff, b[1] * scale_factorbff, b[2] * scale_factorbff,
    #                alpha = 0.7, color='green', arrow_length_ratio=arrow)
    for a, b in zip(twc, bf_vis):
        ax.quiver(a[0], a[1], a[2], b[0] * scale_factorbfv, b[1] * scale_factorbfv, b[2] * scale_factorbfv,
                  alpha = 0.5, color='blue', arrow_length_ratio=arrow)

    
    # pnts_a = onepointlog[:, 1:4]
    # pnts_b = onepointlog[:, 4:7]
    # for a, b in zip(pnts_a, pnts_b):
    #     ax.quiver(a[0], a[1], a[2], b[0] * scale_factor, b[1] * scale_factor, b[2] * scale_factor,
    #               alpha=0.5, color='red', arrow_length_ratio=arrow)

    ax.set_title('FOV quivers')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
     #monte carlo
    # ax.set_xlim([-250, 250])
    # ax.set_ylim([-250, 250])
    # ax.set_zlim([0, 20])
    # ax.set_box_aspect([25, 25, 1])

     #monte carlo fif
    ax.set_xlim([-2.5, 2.5])
    ax.set_ylim([-2.5, 2.5])
    ax.set_zlim([-1, 1])
    ax.set_box_aspect([5, 5, 2])
    ax.legend()
    plt.show()


which_twc = 2
points = np.loadtxt("../Map/two_walls_points_w.csv", delimiter=',')
# points = np.loadtxt("../Map/0_map.csv", delimiter=',')
fov_quivers = np.loadtxt("../Data/0_1_single_run_rotated_quivers.csv", delimiter=',')
BF_quivers = np.loadtxt("../Data/0_1_single_run_brute_force_rotated_quivers.csv", delimiter=',')
mean = np.loadtxt("../Data/mean.csv", delimiter=',')
acc_file= np.loadtxt("../Data/0_1_optimizer_accuracy_file.csv", dtype=float, delimiter=',')
onepointlog= np.loadtxt("../Data/quiversforonepoint.csv", delimiter=',') #id, twc, quiver head, num of features in fov
mask = onepointlog[:,0]==which_twc
# fov_plot(points, fov_quivers, BF_quivers, onepointlog[mask])
fov_plot(points, fov_quivers, BF_quivers, onepointlog, acc_file)

