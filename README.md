# Doosan-A0509

Dự án sử dụng ROS Noetic để điều khiển robot Doosan (Model: A0509) 6 bậc tự do trong bài toán tránh vật cản bằng các thuật toán RRT, RRT*, PRM.

---

## Các bước chạy thuật toán RRT, RRT*, PRM cho Robot thật (Model: A0509)

### 🖥️ Terminal 1

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roscore
```

### 🤖 Terminal 2

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch dsr_launcher dsr_moveit.launch model:=a0509 host:=192.168.137.100 mode:=real
```

### 🧠 Terminal 3

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun my_table_description RRT_hoan_chinh.py
rosrun my_table_description RRT_START_hoan_chinh.py
rosrun my_table_description PRM_hoan_chinh.py
```

---

## 📊 Chạy các đồ thị thống kê  
📁 Thư mục: `~/catkin_ws/src/my_table_description/scripts`

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

./do_thi_RRT.py
./do_thi_PRM.py
./do_thi_RRT_START.py
./do_thi_quy_dao_3_thuat_toan.py
./do_thi_so_sanh_thoi_gian_3_thuat_toan.py
./do_thi_waypoint_3_thuat_toan.py
```

---

## 🔗 Cầu nối từ MoveIt sang thư viện OMPL

Hệ thống sử dụng MoveIt như một cầu nối để tích hợp các thuật toán quy hoạch chuyển động từ OMPL vào môi trường ROS. Việc cấu hình được thực hiện thông qua các file sau:

- Plugin OMPL:  
  ```bash
  code /opt/ros/noetic/share/moveit_planners_ompl/ompl_interface_plugin_description.xml
  ```

- Cấu hình thuật toán:  
  ```bash
  code catkin_ws/src/doosan-robot/moveit_config_a0509/config/ompl_planning.yaml
  ```

Các thuật toán như RRT, RRT*, PRM được cấu hình tại đây để sử dụng trực tiếp trong MoveIt khi điều khiển robot thật.

---

## 🎥 Kết quả thực nghiệm

Video chạy thực nghiệm trên robot **Doosan A0509** tại phòng **LAB Robotics**:  
🔗 [Xem video tại đây](https://drive.google.com/drive/folders/1Q3BMmdfDaVXjt78rGC0YccOj2JgE2R-U)

---

## 👤 Tác giả

**Dinh Vinh** – Robotics & ROS Developer  
📬 Email: dinhvinh20032021@gmail.com
