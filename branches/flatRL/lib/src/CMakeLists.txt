# list headers
SET ( TARGET_HEADER_PATH ${PROJECT_SOURCE_DIR}/lib/include )

# headers
SET ( TARGET_HEADERS
    ${TARGET_HEADER_PATH}/ExecutionGroup.h
    ${TARGET_HEADER_PATH}/Scene.h
    ${TARGET_HEADER_PATH}/SlonEngine.h
)

SET ( TARGET_CONTROL_HEADERS
    ${TARGET_HEADER_PATH}/Control/Control.h
    ${TARGET_HEADER_PATH}/Control/Environment.h
    ${TARGET_HEADER_PATH}/Control/PhysicsControl.h
    ${TARGET_HEADER_PATH}/Control/PhysicsEnvironment.h
)

SET ( TARGET_CONTROL_CHAIN_HEADERS
    ${TARGET_HEADER_PATH}/Control/Chain/Control.h
    ${TARGET_HEADER_PATH}/Control/Chain/Environment.h
    ${TARGET_HEADER_PATH}/Control/Chain/FlatChainEnvironment.h
    ${TARGET_HEADER_PATH}/Control/Chain/HeuristicControl.h
    ${TARGET_HEADER_PATH}/Control/Chain/PDControl.h
    ${TARGET_HEADER_PATH}/Control/Chain/RLControl.h
    ${TARGET_HEADER_PATH}/Control/Chain/FlatRLControl.h
	
)

SET ( TARGET_CONTROL_INVERSE_DYNAMICS_HEADERS
	${TARGET_HEADER_PATH}/Control/InverseDynamics/Control.h
	${TARGET_HEADER_PATH}/Control/InverseDynamics/Environment.h
	${TARGET_HEADER_PATH}/Control/InverseDynamics/RLControl.h
)

SET ( TARGET_CONTROL_HUMANOID_HEADERS
	${TARGET_HEADER_PATH}/Control/Humanoid/Control.h
	${TARGET_HEADER_PATH}/Control/Humanoid/Environment.h
)

SET ( TARGET_CONTROL_PYTHON_HEADERS
	${TARGET_HEADER_PATH}/Control/Python/PythonControl.h
	${TARGET_HEADER_PATH}/Control/Python/PythonEnvironment.h
)

SET ( TARGET_CONTROL_UTILITY_HEADERS
    ${TARGET_HEADER_PATH}/Control/Utility/DirectControl.h
    ${TARGET_HEADER_PATH}/Control/Utility/LooseTimer.h
    ${TARGET_HEADER_PATH}/Control/Utility/EnvironmentWrapper.h
)

SET ( TARGET_LEARNING_HEADERS
    ${TARGET_HEADER_PATH}/Learning/approximator_wrapper.hpp
    ${TARGET_HEADER_PATH}/Learning/config.hpp
    ${TARGET_HEADER_PATH}/Learning/function.hpp
    ${TARGET_HEADER_PATH}/Learning/linear_function.hpp
    ${TARGET_HEADER_PATH}/Learning/neural_network.hpp
    ${TARGET_HEADER_PATH}/Learning/scalar_neural_network.hpp
    ${TARGET_HEADER_PATH}/Learning/radial_basis_function.hpp
    ${TARGET_HEADER_PATH}/Learning/reinforcement_learning.hpp
    ${TARGET_HEADER_PATH}/Learning/reinforcement_learning_wrapper.hpp
    ${TARGET_HEADER_PATH}/Learning/serialization.hpp
    ${TARGET_HEADER_PATH}/Learning/td_lambda_learning.hpp
    #${TARGET_HEADER_PATH}/Learning/BR_learning.hpp
    ${TARGET_HEADER_PATH}/Learning/plain_direct_learning.hpp
    ${TARGET_HEADER_PATH}/Learning/plain_direct_rl.hpp
    ${TARGET_HEADER_PATH}/Learning/natural_actor_critic.hpp
    ${TARGET_HEADER_PATH}/Learning/vector_function.hpp
    ${TARGET_HEADER_PATH}/Learning/iterate_learning.hpp
)

SET ( TARGET_PLOT_HEADERS
    ${TARGET_HEADER_PATH}/Plot/LinePlot.h
    ${TARGET_HEADER_PATH}/Plot/DrawLinePlot.h
    ${TARGET_HEADER_PATH}/Plot/DumpLinePlot.h
)

SET ( TARGET_STATISTICS_HEADERS
    ${TARGET_HEADER_PATH}/Statistics/NumActionsPlot.h
    ${TARGET_HEADER_PATH}/Statistics/Statistics.h
    ${TARGET_HEADER_PATH}/Statistics/ValueFunctionPlot.h
)

SET ( TARGET_UTILITY_HEADERS
    ${TARGET_HEADER_PATH}/Utility/Math.h
    ${TARGET_HEADER_PATH}/Utility/PhysicsDebug.h
    #${TARGET_HEADER_PATH}/Control/Utility/ControlElement.h
)

SOURCE_GROUP( headers FILES                             ${TARGET_HEADERS} )
SOURCE_GROUP( headers\\Control FILES                    ${TARGET_CONTROL_HEADERS} )
SOURCE_GROUP( headers\\Control\\Chain FILES             ${TARGET_CONTROL_CHAIN_HEADERS} )
SOURCE_GROUP( headers\\Control\\InverseDynamics FILES   ${TARGET_CONTROL_INVERSE_DYNAMICS_HEADERS} )
SOURCE_GROUP( headers\\Control\\Humanoid FILES          ${TARGET_CONTROL_HUMANOID_HEADERS} )
SOURCE_GROUP( headers\\Control\\Python FILES            ${TARGET_CONTROL_PYTHON_HEADERS} )
SOURCE_GROUP( headers\\Control\\Utility FILES           ${TARGET_CONTROL_UTILITY_HEADERS} )
SOURCE_GROUP( headers\\Learning FILES                   ${TARGET_LEARNING_HEADERS} )
SOURCE_GROUP( headers\\Plot FILES                       ${TARGET_PLOT_HEADERS} )
SOURCE_GROUP( headers\\Statistics FILES                 ${TARGET_STATISTICS_HEADERS} )
SOURCE_GROUP( headers\\Utility FILES                    ${TARGET_UTILITY_HEADERS} )

SET ( TARGET_HEADERS
    ${TARGET_HEADERS}
    ${TARGET_CONTROL_HEADERS}
    ${TARGET_CONTROL_CHAIN_HEADERS}
	${TARGET_CONTROL_INVERSE_DYNAMICS_HEADERS} 
    ${TARGET_CONTROL_HUMANOID_HEADERS}
    ${TARGET_CONTROL_UTILITY_HEADERS}
    ${TARGET_LEARNING_HEADERS}
    ${TARGET_PLOT_HEADERS}
    ${TARGET_STATISTICS_HEADERS}
    ${TARGET_UTILITY_HEADERS}
)

IF (WALKER_YARD_USE_PYTHON)
	LIST (APPEND TARGET_HEADERS ${TARGET_CONTROL_PYTHON_HEADERS})
ENDIF (WALKER_YARD_USE_PYTHON)

# sources
SET ( TARGET_SOURCES
    ExecutionGroup.cpp
    Scene.cpp
)

SET ( TARGET_CONTROL_SOURCES
	Control/PhysicsControl.cpp
	Control/PhysicsEnvironment.cpp
)

SET ( TARGET_CONTROL_CHAIN_SOURCES
    Control/Chain/Control.cpp
    Control/Chain/Environment.cpp
    Control/Chain/FlatChainEnvironment.cpp
    Control/Chain/HeuristicControl.cpp
    Control/Chain/PDControl.cpp
    Control/Chain/RLControl.cpp
    Control/Chain/FlatRLControl.cpp
)

SET ( TARGET_CONTROL_INVERSE_DYNAMICS_SOURCES
    Control/InverseDynamics/Control.cpp
    Control/InverseDynamics/Environment.cpp
    Control/InverseDynamics/RLControl.cpp
)

SET ( TARGET_CONTROL_HUMANOID_SOURCES
    Control/Humanoid/Control.cpp
    Control/Humanoid/Environment.cpp
)

SET ( TARGET_CONTROL_PYTHON_SOURCES
    Control/Python/PythonControl.cpp
    Control/Python/PythonEnvironment.cpp
)

SET ( TARGET_CONTROL_UTILITY_SOURCES
    Control/Utility/LooseTimer.cpp
)

SET ( TARGET_LEARNING_SOURCES
)

SET ( TARGET_PLOT_SOURCES
    Plot/DrawLinePlot.cpp
    Plot/DumpLinePlot.cpp
)

SET ( TARGET_STATISTICS_SOURCES
)

SET ( TARGET_UTILITY_SOURCES
    Utility/PhysicsDebug.cpp
    #${TARGET_HEADER_PATH}/Control/Utility/ControlElement.h
)

SOURCE_GROUP( sources FILES                             ${TARGET_SOURCES} )
SOURCE_GROUP( sources\\Control FILES                    ${TARGET_CONTROL_SOURCES} )
SOURCE_GROUP( sources\\Control\\Chain FILES             ${TARGET_CONTROL_CHAIN_SOURCES} )
SOURCE_GROUP( sources\\Control\\InverseDynamics FILES   ${TARGET_CONTROL_INVERSE_DYNAMICS_SOURCES} )
SOURCE_GROUP( sources\\Control\\Humanoid FILES          ${TARGET_CONTROL_HUMANOID_SOURCES} )
SOURCE_GROUP( sources\\Control\\Python FILES            ${TARGET_CONTROL_PYTHON_SOURCES} )
SOURCE_GROUP( sources\\Control\\Utility FILES           ${TARGET_CONTROL_UTILITY_SOURCES} )
SOURCE_GROUP( sources\\Learning FILES                   ${TARGET_LEARNING_SOURCES} )
SOURCE_GROUP( sources\\Plot FILES                       ${TARGET_PLOT_SOURCES} )
SOURCE_GROUP( sources\\Statistics FILES                 ${TARGET_STATISTICS_SOURCES} )
SOURCE_GROUP( sources\\Utility FILES                    ${TARGET_UTILITY_SOURCES} )

SET ( TARGET_SOURCES
    ${TARGET_SOURCES}
    ${TARGET_CONTROL_SOURCES}
    ${TARGET_CONTROL_CHAIN_SOURCES}
	${TARGET_CONTROL_INVERSE_DYNAMICS_SOURCES}
    ${TARGET_CONTROL_HUMANOID_SOURCES}
    ${TARGET_CONTROL_UTILITY_SOURCES}
    ${TARGET_LEARNING_SOURCES}
    ${TARGET_PLOT_SOURCES}
    ${TARGET_STATISTICS_SOURCES}
    ${TARGET_UTILITY_SOURCES}
)

IF (WALKER_YARD_USE_PYTHON)
	LIST (APPEND TARGET_SOURCES ${TARGET_CONTROL_PYTHON_SOURCES})
ENDIF (WALKER_YARD_USE_PYTHON)

ADD_LIBRARY(${CONTROLLER_LIBRARY_NAME} STATIC ${TARGET_HEADERS} ${TARGET_SOURCES})
                            
SET (Another_LIBRARIES)
IF (WALKER_YARD_USE_LAPACK)
	LIST (APPEND Another_LIBRARIES ${Lapack_LIBRARIES})
ENDIF (WALKER_YARD_USE_LAPACK)

# link libraries
TARGET_LINK_LIBRARIES ( ${CONTROLLER_LIBRARY_NAME}
	${Boost_LIBRARIES}
	${Another_LIBRARIES}
)

IF (SLON_ENGINE_CONFIGURE_INTRUSIVE)
	ADD_DEPENDENCIES( ${CONTROLLER_LIBRARY_NAME} SlonEngine ${3RD_PARTY_TARGET} )
	TARGET_LINK_LIBRARIES(${CONTROLLER_LIBRARY_NAME} SlonEngine)
ELSE (SLON_ENGINE_CONFIGURE_INTRUSIVE)
	TARGET_LINK_LIBRARIES(${CONTROLLER_LIBRARY_NAME} ${SlonEngine_LIBRARIES})
ENDIF (SLON_ENGINE_CONFIGURE_INTRUSIVE)

# install
#INSTALL ( TARGETS ${TARGET_NAME}
#    RUNTIME DESTINATION ${INSTALL_BINDIR}
#)

