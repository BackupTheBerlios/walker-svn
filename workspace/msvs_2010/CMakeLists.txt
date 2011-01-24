CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
PROJECT (WalkerYard)

# setup 3rd party directory as search path
SET ( CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/3rdParty;${CMAKE_PREFIX_PATH}" )
SET ( 3RD_PARTY_DIRECTORY ${PROJECT_SOURCE_DIR}/3rdParty )
LIST ( APPEND CMAKE_INCLUDE_PATH 
	${3RD_PARTY_DIRECTORY}/boost/ 
	${3RD_PARTY_DIRECTORY}/BULLET_PHYSICS/include
	${3RD_PARTY_DIRECTORY}/DevIL/include
	${3RD_PARTY_DIRECTORY}/GLEW/include
	${3RD_PARTY_DIRECTORY}/SDL/include
)

LIST ( APPEND CMAKE_LIBRARY_PATH 
	${3RD_PARTY_DIRECTORY}/boost/lib
	${3RD_PARTY_DIRECTORY}/BULLET_PHYSICS/lib
	${3RD_PARTY_DIRECTORY}/DevIL/lib
	${3RD_PARTY_DIRECTORY}/GLEW/lib
	${3RD_PARTY_DIRECTORY}/SDL/lib
)

LIST ( APPEND CMAKE_PROGRAM_PATH
    ${3RD_PARTY_DIRECTORY}/gnuplot
)

# copy dlls into target binary dir
ADD_CUSTOM_TARGET (CopyDLL)

MACRO (COPY_DLLS SOURCE DESTINATION COPY_FILES)
    FOREACH(FILENAME ${${COPY_FILES}})
        ADD_CUSTOM_COMMAND( TARGET CopyDLL
                            COMMAND ${CMAKE_COMMAND} -E copy "${SOURCE}/${FILENAME}" "${DESTINATION}/${FILENAME}")
    ENDFOREACH(FILENAME)
ENDMACRO (COPY_DLLS)

SET (BOOST_DLLS
	boost_date_time-vc100-mt-1_43.dll 
    boost_date_time-vc100-mt-gd-1_43.dll 
    boost_filesystem-vc100-mt-1_43.dll
    boost_filesystem-vc100-mt-gd-1_43.dll
    boost_iostreams-vc100-mt-1_43.dll 
    boost_iostreams-vc100-mt-gd-1_43.dll 
    boost_math_c99-vc100-mt-1_43.dll
    boost_math_c99-vc100-mt-gd-1_43.dll
    boost_prg_exec_monitor-vc100-mt-1_43.dll
    boost_prg_exec_monitor-vc100-mt-gd-1_43.dll
    boost_serialization-vc100-mt-1_43.dll
    boost_serialization-vc100-mt-gd-1_43.dll
    boost_signals-vc100-mt-1_43.dll
    boost_signals-vc100-mt-gd-1_43.dll
    boost_system-vc100-mt-1_43.dll
    boost_system-vc100-mt-gd-1_43.dll
    boost_thread-vc100-mt-1_43.dll
    boost_thread-vc100-mt-gd-1_43.dll
)

SET (DEVIL_DLLS
    DevIL.dll
    ILU.dll
    ILUT.dll
    jpeg.dll
    libpng12-0.dll
    libtiff-3.dll
    zlib1.dll
)

SET (GLEW_DLLS
    glew32.dll
    glew32mx.dll
)

SET (SDL_DLLS
    SDL.dll
    SDL_image.dll
)

SET (RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
SET (3RD_PARTY_TARGET_DIRECTORY ${RUNTIME_OUTPUT_DIRECTORY}/${CMAKE_CFG_INTDIR})
COPY_DLLS(${3RD_PARTY_DIRECTORY}/boost/lib  ${3RD_PARTY_TARGET_DIRECTORY}   BOOST_DLLS)
COPY_DLLS(${3RD_PARTY_DIRECTORY}/DevIL/lib  ${3RD_PARTY_TARGET_DIRECTORY}   DEVIL_DLLS)
COPY_DLLS(${3RD_PARTY_DIRECTORY}/GLEW/bin   ${3RD_PARTY_TARGET_DIRECTORY}   GLEW_DLLS)
COPY_DLLS(${3RD_PARTY_DIRECTORY}/SDL/bin    ${3RD_PARTY_TARGET_DIRECTORY}   SDL_DLLS)
	
SET (SIMPLE_GL_CONFIGURE_INTRUSIVE 		ON)	
SET (SIMPLE_GL_PATH						${PROJECT_SOURCE_DIR}/SimpleGL)
SET (XMLPP_CONFIGURE_INTRUSIVE 			ON)
SET (XMLPP_PATH							${PROJECT_SOURCE_DIR}/XML++)
SET (SLON_ENGINE_CONFIGURE_INTRUSIVE 	ON)
SET (SLON_ENGINE_PATH					${PROJECT_SOURCE_DIR}/SlonEngine)
SET (3RD_PARTY_TARGET                   CopyDLL)

ADD_SUBDIRECTORY(SimpleGL)
ADD_SUBDIRECTORY(XML++)
ADD_SUBDIRECTORY(SlonEngine)
ADD_SUBDIRECTORY(WalkerYard)
      