link_directories(${CMAKE_BINARY_DIR}/controllers/iAnt_controller)

add_library(iAnt_controller SHARED iAnt_controller.h iAnt_controller.cpp)
add_library(iAnt_pheromone SHARED iAnt_pheromone.h iAnt_pheromone.cpp)

target_link_libraries(
  iAnt_controller
  iAnt_pheromone
  argos3core_simulator
  argos3plugin_simulator_actuators
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_sensors)
