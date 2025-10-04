#!/usr/bin/env python3
# plot_time_per_goal_bar_only.py
# So sánh thời gian hoàn thành từng goal với 3 bar màu riêng, không có đường nối
import re, os, sys
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

LOG_FILES = {
    'RRT': 'RRT_log.txt',
    'RRT_START': 'RRT_START_log.txt',
    'PRM': 'PRM_log.txt'
}
PAT = re.compile(r'\[trajectory\]\s*\[\s*(\d+)\s*:\s*([+-]?\d+(?:\.\d+)?)\s*:')

def read_seq(path):
    seq = []
    if not os.path.isfile(path):
        return seq
    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            m = PAT.search(line)
            if m:
                seq.append((int(m.group(1)), float(m.group(2))))
            else:
                if '[trajectory]' in line and '[' in line and ']' in line:
                    try:
                        inside = line.split('[',2)[2].split(']')[0]
                        parts = [p.strip() for p in inside.split(':')]
                        if len(parts) >= 2:
                            seq.append((int(parts[0]), float(parts[1])))
                    except Exception:
                        pass
    return seq

def build_idx2group(log_files, n_groups=5):
    all_idxs = set()
    for f in log_files.values():
        for idx,_ in read_seq(f):
            all_idxs.add(idx)
    if not all_idxs:
        return {}
    blocks = np.array_split(np.array(sorted(all_idxs)), min(n_groups, len(all_idxs)))
    idx2g = {}
    for gi, blk in enumerate(blocks, start=1):
        for i in blk:
            idx2g[int(i)] = gi
    return idx2g

def min_time_per_group(seq, idx2g, n_groups=5):
    per_min = {}
    counts = defaultdict(int)
    for idx,t in seq:
        g = idx2g.get(idx)
        if g is None: continue
        counts[g] += 1
        if g not in per_min or t < per_min[g]:
            per_min[g] = t
    times = [per_min.get(g, np.nan) for g in range(1, n_groups+1)]
    ns = [counts.get(g, 0) for g in range(1, n_groups+1)]
    return times, ns

def main():
    n_groups = 5
    idx2g = build_idx2group(LOG_FILES, n_groups)
    if not idx2g:
        print("ERROR: Không có index hợp lệ trong file log.", file=sys.stderr)
        return

    algs = list(LOG_FILES.keys())
    data_times = {}
    data_ns = {}
    for alg in algs:
        seq = read_seq(LOG_FILES[alg])
        times, ns = min_time_per_group(seq, idx2g, n_groups)
        data_times[alg] = times
        data_ns[alg] = ns

    fig, ax = plt.subplots(figsize=(10,6))
    colors = {'RRT':'C0','RRT_START':'C1','PRM':'C2'}

    goals = np.arange(1, n_groups+1)
    width = 0.2  # độ rộng mỗi bar
    x = np.arange(len(goals))

    # vẽ bar cho từng thuật toán
    for i, alg in enumerate(algs):
        times = [data_times[alg][g-1] for g in goals]
        ax.bar(x + i*width, times, width=width, color=colors[alg], label=alg)

    # thêm nhãn thời gian trên mỗi bar
    for i, alg in enumerate(algs):
        times = [data_times[alg][g-1] for g in goals]
        for xi, t in zip(x + i*width, times):
            ax.text(xi, t + 0.05, f"{t:.3f}", ha='center', va='bottom', fontsize=9, color='black')

    ax.set_xticks(x + width)  # đặt nhãn trung bình giữa 3 bar
    ax.set_xticklabels([f"Goal {g}" for g in goals])
    ax.set_ylabel("Time (s)")
    ax.set_title("So sánh thời gian hoàn thành từng goal (3 thuật toán)")
    ax.legend()
    ax.grid(axis='y', linestyle=':', linewidth=0.6)

    plt.tight_layout()
    out = 'time_per_goal_bar_only.png'
    plt.savefig(out, dpi=200)
    print("Saved:", out)
    plt.show()

if __name__ == '__main__':
    main()
