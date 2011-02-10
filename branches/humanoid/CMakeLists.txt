CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
PROJECT (WalkerYard)

# standart
INCLUDE(Configure.cmake)

# vars
SET (TARGET_NAME 		WalkerYard)
SET (TARGET_UNIX_NAME 	WalkerYard)

# sources
ADD_SUBDIRECTORY (src)
IF (WALKER_YARD_BUILD_TESTING)
    ADD_SUBDIRECTORY(Testing)
ENDIF (WALKER_YARD_BUILD_TESTING)

# uninstall rule
INCLUDE (Uninstall OPTIONAL)

