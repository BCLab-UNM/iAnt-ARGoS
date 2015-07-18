# Add the iAnt controller library
add_library(iAnt_controller SHARED iAnt_controller.cpp)

# Add the iAnt pheromone library
add_library(iAnt_pheromone SHARED iAnt_pheromone.cpp)

# Link the controller and pheromone libraries to ARGoS
# Additionally, we link into the generic robot and footbot libraries
#               in order to make use of sensors and actuators
target_link_libraries(iAnt_controller
                      iAnt_pheromone
                      argos3core_simulator
                      argos3plugin_simulator_genericrobot
                      argos3plugin_simulator_footbot)
