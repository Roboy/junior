
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

#################################################
#        Checking for Linux                     #
#################################################
IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
     ADD_DEFINITIONS( -DLINUX )
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

SET( PROJECT_NAME RoboyUtils )
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
FIND_PACKAGE( Boost COMPONENTS thread system filesystem REQUIRED)
FIND_PACKAGE( MVBlueFox REQUIRED )


#################################################
#        INCLUDE DIRECTORIES                    #
################################################
INCLUDE_DIRECTORIES( 
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${MVBLUEFOX_INCLUDE_DIR}
)

#################################################
#        LIBS                                   #
################################################
SET( LIB ${LIB}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${MVBLUEFOX_LIBRARIES}    
    -lasound
)


################################################
#   All component sources must be named here   #
################################################
SET( PROJECT_SRCS
  RoboyFileParser.cpp
  RoboyMediaPlayerInteraction.cpp
  RoboyTransforms.cpp
  RoboyGeometry.cpp   
  RoboyFileSystem.cpp
  RoboyFileWriter.cpp
  RoboyTimer.cpp
  RoboyVideoPlaylist.cpp
	ALSAtestfunctions.cpp
#  RoboyMVBlueFox.cpp
)

ADD_LIBRARY( ${PROJECT_NAME} SHARED ${PROJECT_SRCS} )
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES COMPILER_FLAGS CXX_FLAGS )
TARGET_LINK_LIBRARIES( ${PROJECT_NAME} ${LIB} )

#SET_TARGET_PROPERTIES(
#	${PROJECT_NAME}
#	PROPERTIES
#	DEBUG_POSTFIX d
#)

#TARGET_LINK_LIBRARIES( ${COMP_NAME}Comp ${COMP_NAME} )

