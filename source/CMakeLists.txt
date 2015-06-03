################################################################################
# Compile each iAnt-ARGoS class into a shared object file.
################################################################################
add_library(iAnt_controller SHARED iAnt_controller.cpp)
add_library(iAnt_loop_functions SHARED iAnt_loop_functions.cpp iAnt_qt_user_functions.cpp)
add_library(iAnt_data SHARED iAnt_data.cpp)
add_library(iAnt_qt_user_functions SHARED iAnt_qt_user_functions.cpp)
add_library(iAnt_pheromone SHARED iAnt_pheromone.cpp)

################################################################################
# Correctly link each shared object with its dependencies . . .
################################################################################
target_link_libraries(iAnt_controller
                      iAnt_pheromone
                      argos3core_simulator
                      argos3plugin_simulator_footbot
                      argos3plugin_simulator_genericrobot)

target_link_libraries(iAnt_loop_functions
                      iAnt_controller
                      iAnt_data)

target_link_libraries(iAnt_data
                      iAnt_pheromone
                      argos3core_simulator)

target_link_libraries(iAnt_qt_user_functions
                      iAnt_controller
                      iAnt_pheromone
                      argos3plugin_simulator_qtopengl)

target_link_libraries(iAnt_pheromone)
