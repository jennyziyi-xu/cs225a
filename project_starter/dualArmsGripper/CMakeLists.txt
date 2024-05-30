set(DUALARMSGRIPPER_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DDUALARMSGRIPPER_FOLDER="${DUALARMSGRIPPER_FOLDER}")

# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/dual_arms_gripper_example)
ADD_EXECUTABLE (dualArmGripper 22-dual_arms_keyboard_gripper.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (dualArmGripper ${CS225A_COMMON_LIBRARIES})
