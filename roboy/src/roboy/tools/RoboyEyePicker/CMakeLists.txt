
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

#################################################
#        Checking for Linux                     #
#################################################
IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
     ADD_DEFINITIONS( -DLINUX )
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

SET( PROJECT_NAME RoboyEyePicker )
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


#################################################
#        INCLUDE DIRECTORIES                    #
################################################
#INCLUDE_DIRECTORIES( 
    #${CMAKE_SOURCE_DIR}/src
    #${CMAKE_CURRENT_BINARY_DIR}
#)

#################################################
#        LIBS                                   #
################################################
SET( LIB ${LIB}
    ${OpenCV_LIBS}
    ${CMAKE_BINARY_DIR}/src/roboy/utils/libRoboyUtils.so
)


################################################
#   All component sources must be named here   #
################################################
SET( PROJECT_SRCS
    RoboyEyePicker.cpp
)

ADD_EXECUTABLE( RoboyEyePicker RoboyEyePicker.cpp )
TARGET_LINK_LIBRARIES( RoboyEyePicker ${LIB} )


#ADD_LIBRARY( ${PROJECT_NAME} SHARED ${PROJECT_SRCS} )
#SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES COMPILER_FLAGS CXX_FLAGS )
#TARGET_LINK_LIBRARIES( ${PROJECT_NAME} ${LIB} )



