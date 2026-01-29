cmake_minimum_required(VERSION 3.8)
project(myrobot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 1. Find dependencies
find_package(ament_cmake REQUIRED)
# Lưu ý: rclpy và các gói tin nhắn thường được khai báo trong package.xml là đủ cho gói ament_cmake,
# nhưng giữ ở đây cũng không sao nếu bạn định viết thêm code C++.

# 2. Install Python scripts (Sửa lại tên file cho khớp với file bạn đưa ở đầu)
install(PROGRAMS
  # Đảm bảo đường dẫn này đúng với cấu trúc thư mục của bạn
  # Ví dụ nếu file đặt trong thư mục 'scripts' thì sửa thành scripts/tên_file.py
  excu/control_robot.py  
  excu/p2p.py
  excu/vequydao1.py
  excu/vequydao2.py
  excu/test_one_point.py
  DESTINATION lib/${PROJECT_NAME}
)

# 3. Install Directories (Thư mục quan trọng cho Gazebo Classic)
install(DIRECTORY
  launch
  config
  urdf
  meshes
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# 4. Export dependencies
# Điều này giúp các gói khác tìm thấy tài nguyên của gói này (quan trọng để tìm Mesh)
ament_export_dependencies(ament_cmake)

ament_package()