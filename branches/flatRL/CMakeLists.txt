CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
PROJECT (WalkerYard)

# standart
INCLUDE(Configure.cmake)

# vars
SET (CONTROLLER_LIBRARY_NAME WalkerLib)

ADD_SUBDIRECTORY (lib)
ADD_SUBDIRECTORY (walker)
IF (WALKER_YARD_BUILD_TESTING)
    ADD_SUBDIRECTORY(test)
ENDIF (WALKER_YARD_BUILD_TESTING)

# uninstall rule
# INCLUDE (Uninstall OPTIONAL)

