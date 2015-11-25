###############################################
# define shared object files
###############################################

add_library(DSA_loop_functions MODULE DSA_loop_functions.h
                                      DSA_loop_functions.cpp
                                      DSA_qt_user_functions.h
                                      DSA_qt_user_functions.cpp)

add_library(DSA_controller     SHARED DSA_controller.h
                                      DSA_controller.cpp
									  DSA_loop_functions.h
                                      DSA_loop_functions.cpp)

###############################################
# link shared object files to dependencies
###############################################

target_link_libraries(DSA_controller
					  iAntBaseController)

target_link_libraries(DSA_loop_functions
                      DSA_controller
                      argos3core_simulator
                      argos3plugin_simulator_entities
                      argos3plugin_simulator_footbot
                      argos3plugin_simulator_genericrobot
                      argos3plugin_simulator_qtopengl
                      ${QT_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})