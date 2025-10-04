import pandas as pd
import ast
import matplotlib.pyplot as plt

# Đọc CSV
df = pd.read_csv('trajectory_log.csv')

# Chuyển các cột string -> list
for col in ['positions', 'velocities', 'accelerations']:
    df[col] = df[col].apply(ast.literal_eval)

# Hiển thị thông tin tổng quan
print("=== Thông tin tổng quan CSV ===")
print(f"Số goal: {df['goal_index'].nunique()}")
print(f"Số điểm tổng cộng: {len(df)}")
print(f"Các cột: {df.columns.tolist()}")
print("\n5 dòng đầu tiên:")
print(df.head())

# Vẽ sơ đồ cấu trúc trajectory: mỗi goal là 1 màu
plt.figure(figsize=(10,5))
for goal in df['goal_index'].unique():
    goal_df = df[df['goal_index']==goal]
    plt.scatter(goal_df['point_index'], [goal]*len(goal_df), label=f'Goal {goal}', s=50)
plt.title('Cấu trúc trajectory: point_index vs goal_index')
plt.xlabel('point_index')
plt.ylabel('goal_index')
plt.legend()
plt.grid()
plt.show()

# Kiểm tra số điểm và chiều joint
print("\nSố điểm từng goal:")
for goal in df['goal_index'].unique():
    n_points = len(df[df['goal_index']==goal])
    n_joints = len(df[df['goal_index']==goal]['positions'].iloc[0])
    print(f"Goal {goal}: {n_points} points, {n_joints} joints")
