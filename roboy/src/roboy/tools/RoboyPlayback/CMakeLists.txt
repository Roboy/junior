
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

#################################################
#        Checking for Linux                     #
#################################################
IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
     ADD_DEFINITIONS( -DLINUX )
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

SET( PROJECT_NAME RoboyPlayback)
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
FIND_PACKAGE( Boost COMPONENTS thread system filesystem)
FIND_PACKAGE( PCanUSB REQUIRED )


#################################################
#        INCLUDE DIRECTORIES                    #
################################################
INCLUDE_DIRECTORIES( 
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_BINARY_DIR}
 		${PCAN_USB_INCLUDE_DIR}   
    ${Boost_INCLUDE_DIRS}
		${CMAKE_SOURCE_DIR}/src/roboy/robot
		${CMAKE_SOURCE_DIR}/src/roboy/utils
)

#################################################
#        LIBS                                   #
################################################
SET( LIB ${LIB}
    ${Boost_LIBRARIES}
    ${PCAN_USB_LIBRARY}
    ${CMAKE_BINARY_DIR}/src/roboy/robot/libRoboyRobot.so
    ${CMAKE_BINARY_DIR}/src/roboy/utils/libRoboyUtils.so
)


################################################
#   All component sources must be named here   #
################################################
SET( PROJECT_SRCS
    main.cpp
)

ADD_EXECUTABLE(  ${PROJECT_NAME} main.cpp )
TARGET_LINK_LIBRARIES( ${PROJECT_NAME} ${LIB} )



