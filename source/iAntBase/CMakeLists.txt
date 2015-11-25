###############################################
# define shared object files
###############################################

add_library(iAntBaseController SHARED iAntBaseController.h
                                      iAntBaseController.cpp)

add_library(iAntFood           SHARED iAntFood.h
                                      iAntFood.cpp)

add_library(iAntPheromone      SHARED iAntPheromone.h
                                      iAntPheromone.cpp)

###############################################
# link shared object files to dependencies
###############################################

target_link_libraries(iAntBaseController
                      argos3core_simulator
                      argos3plugin_simulator_footbot
                      argos3plugin_simulator_genericrobot)

target_link_libraries(iAntFood)

target_link_libraries(iAntPheromone)

###############################################
# some notes...
#     argos3core_simulator
#         = required for ARGoS integration
#     argos3plugin_simulator_footbot
#         = required for footbot sensors/actuators
#     argos3plugin_genericrobot
#         = required for general sensors/actuators
###############################################