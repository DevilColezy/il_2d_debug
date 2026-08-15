cmake_minimum_required(VERSION 3.0.2)
project(il_2d_multiscale_debug)

# ── C++17 ────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Qt AUTOMOC (required for the debug_panel Q_OBJECT class) ─────────
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# ── Catkin ───────────────────────────────────────────────────────────
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  message_generation
)

# ── Qt5 Widgets (GUI only; the core library does not depend on Qt) ───
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# ── Eigen3 (types.hpp / core headers use Eigen) ──────────────────────
find_package(Eigen3 REQUIRED)

# ── Custom messages ──────────────────────────────────────────────────
add_message_files(
  FILES
  DebugSnapshot.msg
  PlannerCandidate.msg
)

# ── Custom services ──────────────────────────────────────────────────
add_service_files(
  FILES
  SetPaused.srv
  StepSimulation.srv
  ResetTask.srv
  GenerateScene.srv
  GenerateTask.srv
  SetNavigationGoal.srv
  SetSimSpeed.srv
  ExportFlightLog.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# ── Include directories ──────────────────────────────────────────────
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ── Core C++ library (pure simulation + planning core, no ROS) ───────
set(CORE_SOURCES
  src/truth_esdf_2d.cpp
  src/scene_generator.cpp
  src/connectivity_analyzer.cpp
  src/task_sampler.cpp
  src/fov_raycaster_2d.cpp
  src/observed_grid_2d.cpp
  src/local_planner_30hz.cpp
  src/left_right_route_planner.cpp
  src/effective_target_adapter.cpp
  src/macro_expert_5hz.cpp
  src/hierarchical_expert_fsm.cpp
  src/planar_vehicle_simulator.cpp
  src/debug_simulation.cpp
)

add_library(il_2d_multiscale_core STATIC ${CORE_SOURCES})
add_dependencies(il_2d_multiscale_core
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_options(il_2d_multiscale_core PRIVATE -O2 -Wall -Wextra)
target_link_libraries(il_2d_multiscale_core
  ${catkin_LIBRARIES}
)

# ── debug_simulation_node (simulation + ROS adaptation) ──────────────
add_executable(debug_simulation_node
  src/debug_simulation_node.cpp
)
add_dependencies(debug_simulation_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(debug_simulation_node
  il_2d_multiscale_core
  ${catkin_LIBRARIES}
)

# ── debug_panel_node (Qt5 Widgets debug GUI) ─────────────────────────
# debug_panel.hpp is listed explicitly so CMAKE_AUTOMOC picks up the
# Q_OBJECT class even though it is a header-only-for-moc declaration.
add_executable(debug_panel_node
  src/debug_panel_node.cpp
  src/debug_panel.cpp
  include/il_2d_multiscale_debug/debug_panel.hpp
)
add_dependencies(debug_panel_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(debug_panel_node
  il_2d_multiscale_core
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

# ── Catkin package export ────────────────────────────────────────────
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES il_2d_multiscale_core
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    message_runtime
)

# ── Install ──────────────────────────────────────────────────────────
install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
