################################################################################
# Compile each iAnt-ARGoS class into a shared object file.
################################################################################
add_library(iAnt_controller         MODULE     iAnt_controller.h
                                               iAnt_controller.cpp
                                               iAnt_loop_functions.h
                                               iAnt_loop_functions.cpp
                                               iAnt_pheromone.h
                                               iAnt_pheromone.cpp)

add_library(DSA_controller          MODULE     DSA_controller.h
                                               DSA_controller.cpp                                               iAnt_loop_functions.h
                                               iAnt_loop_functions.cpp                                               iAnt_pheromone.h
                                               iAnt_pheromone.cpp)

add_library(iAnt_loop_functions     MODULE     iAnt_loop_functions.h
                                               iAnt_loop_functions.cpp
                                               iAnt_controller.h
                                               iAnt_controller.cpp
                                               DSA_controller.h
                                               DSA_controller.cpp
                                               iAnt_qt_user_functions.h
                                               iAnt_qt_user_functions.cpp
                                               iAnt_pheromone.h
                                               iAnt_pheromone.cpp)

# add_library(DSA_loop_functions      MODULE     DSA_loop_functions.h
#                                                DSA_loop_functions.cpp
#                                                DSA_controller.h
#                                                DSA_controller.cpp
#                                                iAnt_qt_user_functions.h
#                                                iAnt_qt_user_functions.cpp)

################################################################################
# Correctly link each shared object with its dependencies . . .
################################################################################

target_link_libraries(iAnt_controller
                      argos3core_simulator
                      argos3plugin_simulator_entities
                      argos3plugin_simulator_footbot
                      argos3plugin_simulator_genericrobot)

target_link_libraries(DSA_controller
                      argos3core_simulator
                      argos3plugin_simulator_entities
                      argos3plugin_simulator_footbot
                      argos3plugin_simulator_genericrobot)

target_link_libraries(iAnt_loop_functions
                      argos3core_simulator
                      argos3plugin_simulator_entities
                      argos3plugin_simulator_footbot
                      argos3plugin_simulator_genericrobot
                      argos3plugin_simulator_qtopengl
                      ${QT_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})

# target_link_libraries(DSA_loop_functions
#                       argos3plugin_simulator_entities
#                       argos3plugin_simulator_footbot
#                       argos3plugin_simulator_genericrobot
#                       argos3plugin_simulator_qtopengl
#                       ${QT_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})