# flags
SET(BUILD_SHARED_LIBS ON)

# standart
SET (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}")
INCLUDE (StandartIncludes)
INCLUDE (FindNecessaryLibraries.cmake)
INCLUDE (OptionDependentOnPackage)

# check dependencies
IF (NOT BULLET_FOUND)
	MESSAGE(FATAL_ERROR "Can't compile Walker without Bullet")
ENDIF (NOT BULLET_FOUND)

# documentation
OPTION_DEPENDENT_ON_PACKAGE (BUILD_DOCUMENTATION "Set to ON to build doxygen reference documentation" DOXYGEN_FOUND)
MESSAGE ("Build documentation: " ${BUILD_DOCUMENTATION})

# sse optimization
OPTION (WALKER_YARD_USE_SSE "Set to ON to enable sse optimizations" ON)
MESSAGE ("Use SSE: " ${WALKER_YARD_USE_SSE})

# build testing
OPTION (WALKER_YARD_BUILD_TESTING "Set to ON to build testing projects" ON)
MESSAGE ("Build testing projects: " ${WALKER_YARD_BUILD_TESTING})

# sse optimization
OPTION (WALKER_YARD_USE_PYTHON "Set to ON to enable python scripts for WalkerYard" OFF)
MESSAGE ("Use Python: " ${WALKER_YARD_USE_PYTHON})

# lapack library
OPTION (WALKER_YARD_USE_LAPACK "Set to ON to enable lapack library for WalkerYard" OFF)
MESSAGE ("Use Lapack: " ${WALKER_YARD_USE_LAPACK})

# check for sse
IF (WALKER_YARD_USE_SSE)
    SET (SSE_VERSION 3) 

    # determine sse version
    IF (${SSE_VERSION} GREATER 3)
        SET (WALKER_YARD_USE_SSE4 ON)
    ENDIF (${SSE_VERSION} GREATER 3)
    IF (${SSE_VERSION} GREATER 2)
        SET (WALKER_YARD_USE_SSE3 ON)
    ENDIF (${SSE_VERSION} GREATER 2)
    IF (${SSE_VERSION} GREATER 1)
        SET (WALKER_YARD_USE_SSE2 ON)
    ENDIF (${SSE_VERSION} GREATER 1)

    MESSAGE ("Used sse version: " ${SSE_VERSION})

ENDIF (WALKER_YARD_USE_SSE)

IF (WALKER_YARD_USE_LAPACK)
	SET (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:LIBCMT /NODEFAULTLIB:LIBCMTD")
	ADD_DEFINITIONS (/D "BIND_FORTRAN_LOWERCASE_UNDERSCORE")
ENDIF (WALKER_YARD_USE_LAPACK)

# config
CONFIGURE_FILE (${PROJECT_SOURCE_DIR}/lib/include/Config.h.cmake ${PROJECT_SOURCE_DIR}/lib/include/Config.h)

# enable sse for g++
IF (CMAKE_COMPILER_IS_GNUCC AND WALKER_YARD_USE_SSE)
	ADD_DEFINITIONS (-mmmx)
	ADD_DEFINITIONS (-msse)
	ADD_DEFINITIONS (-mfpmath=sse)

    IF (WALKER_YARD_USE_SSE2)
	    ADD_DEFINITIONS (-msse2)
    ENDIF (WALKER_YARD_USE_SSE2)

    IF (WALKER_YARD_USE_SSE3)
	    ADD_DEFINITIONS (-msse3)
    ENDIF (WALKER_YARD_USE_SSE3)

    IF (WALKER_YARD_USE_SSE4)
	    ADD_DEFINITIONS (-msse4)
    ENDIF (WALKER_YARD_USE_SSE4)
ENDIF (CMAKE_COMPILER_IS_GNUCC AND WALKER_YARD_USE_SSE)

IF (BUILD_DOCUMENTATION)
    SET (USER_DOX_PROJECT_FOLDER "Utility")
    INCLUDE(Doxy)
ENDIF (BUILD_DOCUMENTATION)

IF (SLON_ENGINE_CONFIGURE_INTRUSIVE)
	SET (SlonEngine_INCLUDE_DIR ${SLON_ENGINE_PATH} ${SIMPLE_GL_PATH} ${XMLPP_PATH})
ENDIF (SLON_ENGINE_CONFIGURE_INTRUSIVE)

INCLUDE_DIRECTORIES ( 
    ${PROJECT_SOURCE_DIR}/lib/include 
    ${SLON_ENGINE_INCLUDE_DIR}
	${Boost_INCLUDE_DIR}
    ${SlonEngine_INCLUDE_DIR}
)
