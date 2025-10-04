import re
import numpy as np
import matplotlib.pyplot as plt
import math

log_file = "PRM_log.txt"

# Regex lấy time và 6 joint
traj_pattern = re.compile(
    r"\[trajectory\]\s+\[\d+\s*:\s*([\d\.]+)\s*:\s*[-\d\.]+\]\s+([- \d\.]+)"
)
goal_pattern = re.compile(r"callback: Trajectory received")

segments = []  # mỗi segment = {"times": [], "joints": [[],...,[]]}
current = None

with open(log_file, encoding="utf-8") as f:
    for line in f:
        if goal_pattern.search(line):
            # bắt đầu 1 goal mới
            if current and len(current["times"]) > 0:
                segments.append(current)
            current = {"times": [], "joints": [[] for _ in range(6)]}
        else:
            m = traj_pattern.search(line)
            if m and current:
                t = float(m.group(1))
                values = list(map(float, m.group(2).split()))
                if len(values) == 6:
                    current["times"].append(t)
                    for i in range(6):
                        current["joints"][i].append(values[i] * math.pi / 180.0)

if current and len(current["times"]) > 0:
    segments.append(current)

print(f"Đã tách {len(segments)} goal trajectory từ log.")

# ===== Vẽ 6 Figure riêng biệt =====
for j in range(6):
    fig, axes = plt.subplots(2, 1, figsize=(8,6), sharex=True, num=f"Khớp {j+1}")

    # --- Đồ thị vị trí ---
    for idx, seg in enumerate(segments):
        times = np.array(seg["times"])
        vals = np.array(seg["joints"][j])
        axes[0].plot(times, vals, label=f"Goal {idx+1}")
        if len(times) > 0:
            axes[0].scatter(times[0], vals[0], c="green", marker="o")  # start
            axes[0].scatter(times[-1], vals[-1], c="red", marker="x")  # end
    axes[0].set_ylabel(f"Góc (rad)")
    axes[0].set_title(f"Khớp {j+1}")
    axes[0].legend()
    axes[0].grid(True)

    # --- Đồ thị vận tốc ---
    for idx, seg in enumerate(segments):
        times = np.array(seg["times"])
        vals = np.array(seg["joints"][j])
        if len(times) > 1:
            vel = np.gradient(vals, times)
            axes[1].plot(times, vel, label=f"Goal {idx+1}")
            axes[1].scatter(times[0], vel[0], c="green", marker="o")
            axes[1].scatter(times[-1], vel[-1], c="red", marker="x")
    axes[1].set_xlabel("Thời gian (s)")
    axes[1].set_ylabel("Vận tốc (rad/s)")
    axes[1].grid(True)
    
     # --- Thêm tiêu đề chính giữa trên cùng ---
    fig.suptitle("Thuật toán PRM", fontsize=14, fontweight="bold")

    plt.tight_layout(rect=[0, 0, 1, 0.95])  # chừa chỗ cho suptitle


# Chỉ gọi show() 1 lần, matplotlib sẽ mở tất cả figure cùng lúc
plt.show()
