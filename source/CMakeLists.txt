################################################################################
# Compile each iAnt-ARGoS class into a shared object file.
################################################################################
add_library(iAnt_controller MODULE iAnt_controller.h
                                   iAnt_controller.cpp
                                   iAnt_loop_functions.h
                                   iAnt_loop_functions.cpp)

add_library(iAnt_loop_functions MODULE iAnt_loop_functions.h
                                       iAnt_loop_functions.cpp
                                       iAnt_qt_user_functions.h
                                       iAnt_qt_user_functions.cpp
                                       iAnt_data.h
                                       iAnt_data.cpp
                                       iAnt_controller.h
                                       iAnt_controller.cpp)

add_library(iAnt_data SHARED iAnt_data.h
                             iAnt_data.cpp)

add_library(iAnt_pheromone SHARED iAnt_pheromone.h
                                  iAnt_pheromone.cpp)

################################################################################
# Correctly link each shared object with its dependencies . . .
################################################################################
target_link_libraries(iAnt_controller
                      iAnt_data
		      iAnt_pheromone
                      argos3core_simulator
                      argos3plugin_simulator_footbot
                      argos3plugin_simulator_genericrobot)

target_link_libraries(iAnt_loop_functions
                      iAnt_data
		      iAnt_pheromone
                      argos3core_simulator
                      argos3plugin_simulator_footbot
                      argos3plugin_simulator_genericrobot
                      argos3plugin_simulator_qtopengl)

target_link_libraries(iAnt_data
                      iAnt_pheromone
                      argos3core_simulator)

target_link_libraries(iAnt_pheromone)