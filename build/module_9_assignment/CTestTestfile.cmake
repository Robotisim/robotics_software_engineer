# CMake generated Testfile for 
# Source directory: /home/kushal16/rse_ws/src/robotics_software_engineer/module_9_assignment
# Build directory: /home/kushal16/rse_ws/src/robotics_software_engineer/build/module_9_assignment
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(lint_cmake "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/kushal16/rse_ws/src/robotics_software_engineer/build/module_9_assignment/test_results/module_9_assignment/lint_cmake.xunit.xml" "--package-name" "module_9_assignment" "--output-file" "/home/kushal16/rse_ws/src/robotics_software_engineer/build/module_9_assignment/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/humble/bin/ament_lint_cmake" "--xunit-file" "/home/kushal16/rse_ws/src/robotics_software_engineer/build/module_9_assignment/test_results/module_9_assignment/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/kushal16/rse_ws/src/robotics_software_engineer/module_9_assignment" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;47;ament_add_test;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;21;ament_lint_cmake;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/kushal16/rse_ws/src/robotics_software_engineer/module_9_assignment/CMakeLists.txt;26;ament_package;/home/kushal16/rse_ws/src/robotics_software_engineer/module_9_assignment/CMakeLists.txt;0;")
add_test(xmllint "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/kushal16/rse_ws/src/robotics_software_engineer/build/module_9_assignment/test_results/module_9_assignment/xmllint.xunit.xml" "--package-name" "module_9_assignment" "--output-file" "/home/kushal16/rse_ws/src/robotics_software_engineer/build/module_9_assignment/ament_xmllint/xmllint.txt" "--command" "/opt/ros/humble/bin/ament_xmllint" "--xunit-file" "/home/kushal16/rse_ws/src/robotics_software_engineer/build/module_9_assignment/test_results/module_9_assignment/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/kushal16/rse_ws/src/robotics_software_engineer/module_9_assignment" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/kushal16/rse_ws/src/robotics_software_engineer/module_9_assignment/CMakeLists.txt;26;ament_package;/home/kushal16/rse_ws/src/robotics_software_engineer/module_9_assignment/CMakeLists.txt;0;")