INCLUDE_DIRECTORIES ( 
    ${TARGET_HEADER_PATH} 
    ${SlonEngine_INCLUDE_DIR}
)

# list headers
SET (CAR_TARGET_NAME "Car")

# headers
SET ( CAR_TARGET_HEADERS
    ${TARGET_LEARNING_HEADERS}
    CarLearning.h
)

# sources
SET ( CAR_TARGET_SOURCES
	main.cpp
)

ADD_EXECUTABLE( ${CAR_TARGET_NAME} ${CAR_TARGET_HEADERS} ${CAR_TARGET_SOURCES} )

