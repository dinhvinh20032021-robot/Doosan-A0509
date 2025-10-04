#!/usr/bin/env python3
# plot_num_points_per_goal_exact.py
# Đếm số waypoint mỗi lần plan (idx reset về 0) và vẽ bar chart

import os, re, sys
import numpy as np
import matplotlib.pyplot as plt

LOG_FILES = {
    'RRT': 'RRT_log.txt',
    'RRT_START': 'RRT_START_log.txt',
    'PRM': 'PRM_log.txt'
}
PAT = re.compile(r'\[trajectory\]\s*\[\s*(\d+)\s*:\s*([+-]?\d+(?:\.\d+)?)')

def read_waypoints_per_goal(path):
    """Đọc file log và trả về list [số waypoint goal1, goal2, ...]"""
    seq = []
    if not os.path.isfile(path):
        return seq
    waypoints = []
    last_idx = -1
    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            m = PAT.search(line)
            if not m:
                continue
            idx = int(m.group(1))
            # Nếu index quay về 0 thì đóng 1 goal cũ
            if idx == 0 and waypoints:
                seq.append(len(waypoints))
                waypoints = []
            waypoints.append(idx)
        # goal cuối chưa append
        if waypoints:
            seq.append(len(waypoints))
    return seq

def main():
    algs = list(LOG_FILES.keys())
    data_ns = {}
    max_goals = 0
    for alg in algs:
        ns = read_waypoints_per_goal(LOG_FILES[alg])
        data_ns[alg] = ns
        max_goals = max(max_goals, len(ns))

    if max_goals == 0:
        print("ERROR: Không đọc được goal nào từ log.", file=sys.stderr)
        return

    fig, ax = plt.subplots(figsize=(10,6))
    colors = {'RRT':'C0','RRT_START':'C1','PRM':'C2'}
    goals = np.arange(1, max_goals+1)
    width = 0.2
    x = np.arange(len(goals))

    for i, alg in enumerate(algs):
        ns = data_ns[alg]
        # bổ sung NaN nếu thuật toán có ít goal hơn
        ns = ns + [np.nan]*(max_goals - len(ns))
        ax.bar(x + i*width, ns, width=width, color=colors[alg], label=alg)
        for xi, n in zip(x + i*width, ns):
            if not np.isnan(n):
                ax.text(xi, n + 0.5, str(n), ha='center', va='bottom', fontsize=9, color='black')

    ax.set_xticks(x + width)
    ax.set_xticklabels([f"Goal {g}" for g in goals])
    ax.set_ylabel("Số waypoint")
    ax.set_title("Số waypoint mỗi lần plan (3 thuật toán)")
    ax.legend()
    ax.grid(axis='y', linestyle=':', linewidth=0.6)

    plt.tight_layout()
    out = 'num_points_per_goal_exact.png'
    plt.savefig(out, dpi=200)
    print("Saved:", out)
    plt.show()

if __name__ == '__main__':
    main()
