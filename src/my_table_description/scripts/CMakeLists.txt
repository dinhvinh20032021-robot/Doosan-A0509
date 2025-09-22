## ----------------------------------------------------------------
## Cài đặt script publish_table.py để ROS có thể chạy được
install(PROGRAMS
  scripts/publish_table.py
  DESTINATION \${CATKIN_PACKAGE_BIN_DESTINATION}
)
## ----------------------------------------------------------------
