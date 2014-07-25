
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

#################################################
#        Checking for Linux                     #
#################################################
IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
     ADD_DEFINITIONS( -DLINUX )
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

SET( PROJECT_NAME RoboyFaceTrainer)
PROJECT ( ${PROJECT_NAME} )

#################################################
#        GLOBAL SET COMMANDS                    #
#################################################
#IF (${CMAKE_BUILD_TYPE} STREQUAL "Release")
#    SET(CMAKE_VERBOSE_MAKEFILE OFF)
#ELSEIF (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
#    SET(CMAKE_VERBOSE_MAKEFILE ON)
#ENDIF (${CMAKE_BUILD_TYPE} STREQUAL "Release")

SET(CMAKE_MODULE_PATH
    ${CMAKE_SOURCE_DIR}/modules
)


#################################################
#        REQUIRED PACKAGES                      #
#################################################
FIND_PACKAGE( OpenCV REQUIRED )
FIND_PACKAGE( Boost COMPONENTS thread system filesystem)
FIND_PACKAGE( MVBlueFox REQUIRED )


#################################################
#        INCLUDE DIRECTORIES                    #
################################################
INCLUDE_DIRECTORIES( 
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_BINARY_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${MVBLUEFOX_INCLUDE_DIR}
	${CMAKE_SOURCE_DIR}/src/roboy/datastructures
	${CMAKE_SOURCE_DIR}/src/roboy/utils
	${CMAKE_SOURCE_DIR}/src/roboy/vision
)

#################################################
#        LIBS                                   #
################################################
SET( LIB ${LIB}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${MVBLUEFOX_LIBRARIES}    
    ${CMAKE_BINARY_DIR}/src/roboy/datastructures/libRoboyDataStructures.so
    ${CMAKE_BINARY_DIR}/src/roboy/utils/libRoboyUtils.so
    ${CMAKE_BINARY_DIR}/src/roboy/vision/libRoboyVision.so
)


################################################
#   All component sources must be named here   #
################################################
SET( PROJECT_SRCS
    RoboyFaceTrainer.cpp
)

ADD_EXECUTABLE( RoboyFaceTrainer RoboyFaceTrainer.cpp )
TARGET_LINK_LIBRARIES( RoboyFaceTrainer ${LIB} )


