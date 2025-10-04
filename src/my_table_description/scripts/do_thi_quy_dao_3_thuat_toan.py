#!/usr/bin/env python3
# Hiển thị 3 cửa sổ (tab) riêng, xếp song song nếu GUI backend hỗ trợ.
import os, re
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import cm

# ----------------- CẤU HÌNH -----------------
LOG_FILES = {
    'PRM': 'PRM_log.txt',
    'RRT': 'RRT_log.txt',
    'RRT*': 'RRT_START_log.txt'
}
DEG2RAD = True
OUT_PNG = {
    'PRM': 'robot_PRM.png',
    'RRT': 'robot_RRT.png',
    'RRT*': 'robot_RRT_star.png'
}
WINDOW_SIZE = (640, 600)   # width, height (px) - dùng khi di chuyển cửa sổ
GAP = 10                   # khoảng cách giữa các cửa sổ
# ---------------------------------------------

# DH params Doosan A0509
DH_params = [
    [0,        0,     0.275, 0],
    [-np.pi/2, 0,     0,     0],
    [0,        0.41,  0,     0],
    [-np.pi/2, 0,     0.15,  0],
    [np.pi/2,  0,     0,     0],
    [-np.pi/2, 0,     0.065, 0]
]

def dh_transform(alpha, a, d, theta):
    sa, ca = np.sin(alpha), np.cos(alpha)
    st, ct = np.sin(theta), np.cos(theta)
    return np.array([[ct, -st*ca, st*sa, a*ct],
                     [st,  ct*ca, -ct*sa, a*st],
                     [0,      sa,     ca,   d],
                     [0,       0,      0,   1]])

def forward_kinematics(joints_rad):
    T = np.eye(4)
    for i in range(6):
        alpha, a, d, offset = DH_params[i]
        theta = joints_rad[i] + offset
        T = T @ dh_transform(alpha, a, d, theta)
    return T[0,3], T[1,3], T[2,3]

float_re = re.compile(r'[-+]?\d*\.\d+|[-+]?\d+')

def parse_log_file(path):
    times = []
    positions = []
    if not os.path.exists(path):
        print(f"[WARN] file not found: {path}")
        return np.array([]), np.array([])
    with open(path, 'r') as f:
        for line in f:
            if '[trajectory]' not in line:
                continue
            nums = float_re.findall(line)
            if len(nums) < 8:
                continue
            try:
                t = float(nums[1])
                joints_vals = list(map(float, nums[-6:]))
            except:
                continue
            if DEG2RAD:
                joints_rad = np.radians(joints_vals)
            else:
                joints_rad = np.array(joints_vals, dtype=float)
            try:
                x,y,z = forward_kinematics(joints_rad)
            except:
                continue
            times.append(t)
            positions.append([x,y,z])
    if not positions:
        return np.array([]), np.array([])
    times = np.array(times); positions = np.array(positions)
    order = np.argsort(times)
    return times[order], positions[order]

def plot_colored_segments(ax, times, pos, cmap_name):
    if len(times) < 2:
        return
    cmap = cm.get_cmap(cmap_name)
    tmin, tmax = times.min(), times.max()
    denom = (tmax - tmin) if tmax != tmin else 1.0
    for i in range(len(times)-1):
        c = cmap((times[i]-tmin)/denom)
        seg = pos[i:i+2]
        ax.plot(seg[:,0], seg[:,1], seg[:,2], color=c, linewidth=2)

# Thử di chuyển cửa sổ (nếu backend hỗ trợ)
def move_window(fig, x, y, w, h):
    backend = matplotlib.get_backend()
    mgr = fig.canvas.manager
    try:
        if 'Qt' in backend:
            # Qt backend
            try:
                mgr.window.setGeometry(x, y, w, h)
            except:
                mgr.window.move(x, y)
        elif 'TkAgg' in backend:
            # Tk backend
            try:
                mgr.window.wm_geometry(f"{w}x{h}+{x}+{y}")
            except:
                pass
        elif 'WX' in backend:
            try:
                mgr.window.SetPosition((x,y))
            except:
                pass
    except Exception:
        pass  # không quan trọng nếu không di chuyển được

def create_and_show_window(algo, times, pos, cmap_name, position_index):
    fig = plt.figure(figsize=(WINDOW_SIZE[0]/100, WINDOW_SIZE[1]/100))
    ax = fig.add_subplot(111, projection='3d')
    if pos.size == 0:
        ax.text(0.5,0.5,0.5, "No data", transform=ax.transAxes, ha='center')
    else:
        plot_colored_segments(ax, times, pos, cmap_name)
        ax.scatter(pos[0,0], pos[0,1], pos[0,2], color='k', marker='o', s=50)
        ax.scatter(pos[-1,0], pos[-1,1], pos[-1,2], color='r', marker='^', s=50)
        ax.text(pos[-1,0], pos[-1,1], pos[-1,2], f' {algo}', fontsize=9, weight='bold')
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    # save individual PNG
    out = OUT_PNG.get(algo, f'robot_{algo}.png')
    plt.tight_layout()
    plt.savefig(out, dpi=200)
    print(f"[INFO] saved {out} ({len(times)} points)")
    # try to move window
    # compute x,y based on index
    x = position_index * (WINDOW_SIZE[0] + GAP)
    y = 40  # small offset from top
    try:
        move_window(fig, x, y, WINDOW_SIZE[0], WINDOW_SIZE[1])
    except:
        pass
    return fig

def main():
    figs = []
    backend = matplotlib.get_backend()
    print(f"[INFO] Matplotlib backend: {backend}")
    # parse all
    datas = {}
    for algo,path in LOG_FILES.items():
        t,p = parse_log_file(path)
        datas[algo] = (t,p)
        if p.size == 0:
            print(f"[WARN] No data for {algo} (file: {path})")
        else:
            print(f"[INFO] {algo}: {len(t)} points, time [{t.min():.3f}, {t.max():.3f}]")

    # create three separate windows
    idx = 0
    for algo in LOG_FILES.keys():
        times, pos = datas[algo]
        fig = create_and_show_window(algo, times, pos, {'PRM':'Blues','RRT':'Greens','RRT*':'Oranges'}[algo], idx)
        figs.append(fig)
        idx += 1

    if all(datas[a][1].size==0 for a in LOG_FILES.keys()):
        print("[ERROR] Không có dữ liệu ở tất cả file log. Kết thúc.")
        return

    # show all figures (blocking)
    plt.show()

if __name__ == '__main__':
    main()
