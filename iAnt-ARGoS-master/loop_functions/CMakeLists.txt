# Add the loop functions and qt user functions library
add_library(iAnt_loop_functions SHARED iAnt_loop_functions.cpp
                                       iAnt_qt_user_functions.cpp)

# Link the loop functions library to the controller
target_link_libraries(iAnt_loop_functions
                      iAnt_controller
                      argos3plugin_simulator_qtopengl)