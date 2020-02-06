# oroca_ros_tutorials_gui
mavros 메세지를 통해 드론의 정보를 받아와 표시하고 키입력으로 드론의 상태를 제어할 수 있는 gui(학부연구생 과제) 

# 1.설치 방법
  1. catkin workspace에서 git clone https://github.com/0307kwon/oroca_ros_tutorials_gui
  
  2. catkin_make_isolated로 빌드
  
  3. qcustomplot.hpp 오류가 뜨면 소스파일의 include/oroca_ros_tutorials_gui/ 에 있는 qcustomplot.hpp 파일을
  
  필요로 하는 위치에 복사, 붙여넣기
  
  
# 2.실행 방법
  터미널에서 rosrun oroca_ros_tutorials_gui oroca_ros_tutorials_gui
  
