
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

#################################################
#        Checking for Linux                     #
#################################################
IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
     ADD_DEFINITIONS( -DLINUX )
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

SET( PROJECT_NAME RoboyFileParserTester)
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
# FIND_PACKAGE( OpenCV REQUIRED )


#################################################
#        INCLUDE DIRECTORIES                    #
################################################
#INCLUDE_DIRECTORIES( 
#    ${CMAKE_SOURCE_DIR}/src
#    ${CMAKE_CURRENT_BINARY_DIR}
#	${RTM_INCLUDE_DIR}
#	${RTM_INCLUDE_DIR}/idl
#	${LIB_ECCEROBOT_INCLUDE_DIR}
#	${LIB_STATS_INCLUDE_DIR}
#)

#################################################
#        LIBS                                   #
################################################
SET( LIB ${LIB}
#    ${OpenCV_LIBS}
    ${CMAKE_BINARY_DIR}/src/roboy/utils/libRoboyUtils.so
)


################################################
#   All component sources must be named here   #
################################################
SET( PROJECT_SRCS
    RoboyFileParserTester.cpp
)

ADD_EXECUTABLE( RoboyFileParserTester ${PROJECT_SRCS} )
TARGET_LINK_LIBRARIES( RoboyFileParserTester ${LIB} )


