link_directories(${CMAKE_BINARY_DIR}/controllers/iAnt_controller)
set(iAnt_loop_functions_SOURCES iAnt_loop_functions.cpp)

if(ARGOS_COMPILE_QTOPENGL)
  set(iAnt_loop_functions_SOURCES
    ${iAnt_loop_functions_SOURCES}
    iAnt_qt_user_functions.cpp)
endif(ARGOS_COMPILE_QTOPENGL)

add_library(iAnt_loop_functions MODULE ${iAnt_loop_functions_SOURCES})
target_link_libraries(iAnt_loop_functions
  iAnt_controller
  argos3core_simulator
  argos3plugin_simulator_actuators
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media
  argos3plugin_simulator_sensors)

if(ARGOS_COMPILE_QTOPENGL)
target_link_libraries(iAnt_loop_functions argos3plugin_simulator_qtopengl)
endif(ARGOS_COMPILE_QTOPENGL)
