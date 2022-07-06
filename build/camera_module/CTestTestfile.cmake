# CMake generated Testfile for 
# Source directory: /home/rover/WinterResearch/src/camera_module
# Build directory: /home/rover/WinterResearch/build/camera_module
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_camera_module_venv_check_camera_module-requirements "/home/rover/WinterResearch/build/camera_module/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/rover/WinterResearch/build/camera_module/test_results/camera_module/venv_check-camera_module-requirements.xml" "--working-dir" "/home/rover/WinterResearch/build/camera_module" "--return-code" "/home/rover/WinterResearch/build/camera_module/catkin_generated/env_cached.sh rosrun catkin_virtualenv venv_check venv --requirements /home/rover/WinterResearch/src/camera_module/requirements.txt         --extra-pip-args \"\\\"-qq --retries 10 --timeout 30\\\"\"         --xunit-output /home/rover/WinterResearch/build/camera_module/test_results/camera_module/venv_check-camera_module-requirements.xml")
set_tests_properties(_ctest_camera_module_venv_check_camera_module-requirements PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/melodic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/melodic/share/catkin_virtualenv/cmake/catkin_generate_virtualenv.cmake;157;catkin_run_tests_target;/home/rover/WinterResearch/src/camera_module/CMakeLists.txt;27;catkin_generate_virtualenv;/home/rover/WinterResearch/src/camera_module/CMakeLists.txt;0;")
subdirs("gtest")
