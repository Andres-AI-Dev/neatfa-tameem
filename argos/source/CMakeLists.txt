################################################################################
# Compile each iAnt-ARGoS class into a shared object file.
################################################################################
add_library(iAnt_controller     SHARED iAnt_controller.h
                                    iAnt_controller.cpp
                                    iAnt_loop_functions.h
                                    iAnt_loop_functions.cpp
                                    iAnt_pheromone.h
                                    iAnt_pheromone.cpp)

add_library(iAnt_rl_controller  SHARED iAnt_rl_controller.h
                                    iAnt_rl_controller.cpp
                                    iAnt_controller.h
                                    iAnt_controller.cpp
                                    iAnt_pheromone.h
                                    iAnt_pheromone.cpp)

add_library(iAnt_loop_functions SHARED iAnt_loop_functions.h
                                    iAnt_loop_functions.cpp
                                    iAnt_controller.h
                                    iAnt_controller.cpp
                                    iAnt_qt_user_functions.h
                                    iAnt_qt_user_functions.cpp
                                    iAnt_pheromone.h
                                    iAnt_pheromone.cpp
                                    main.cpp)

# Removed BasicGA and NN/GA libraries

################################################################################
# Correctly link each shared object with its dependencies . . .
################################################################################

add_executable(iant_main main.cpp)

target_link_libraries(iAnt_controller
                      argos3core_simulator
                      argos3plugin_simulator_entities
                      argos3plugin_simulator_footbot
                      argos3plugin_simulator_genericrobot)

target_link_libraries(iAnt_rl_controller
                      argos3core_simulator
                      argos3plugin_simulator_entities
                      argos3plugin_simulator_footbot
                      argos3plugin_simulator_genericrobot
                      ${JSONCPP_LIBRARIES})

target_link_libraries(iAnt_loop_functions
                      argos3core_simulator
                      argos3plugin_simulator_entities
                      argos3plugin_simulator_footbot
                      argos3plugin_simulator_genericrobot
                      argos3plugin_simulator_qtopengl)

target_link_libraries(iant_main
	iAnt_loop_functions
        iAnt_controller
        argos3core_simulator
        )

install(TARGETS iAnt_controller iAnt_rl_controller iAnt_loop_functions iant_main
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
