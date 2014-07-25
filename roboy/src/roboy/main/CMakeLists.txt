
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

#################################################
#        Checking for Linux                     #
#################################################
IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
     ADD_DEFINITIONS( -DLINUX )
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

SET( PROJECT_NAME RoboyStateMachine )
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
FIND_PACKAGE( Boost COMPONENTS system thread filesystem REQUIRED)
FIND_PACKAGE( ALUT REQUIRED )
FIND_PACKAGE( MVBlueFox REQUIRED )
FIND_PACKAGE( PCanUSB REQUIRED )

#################################################
#        INCLUDE DIRECTORIES                    #
################################################
INCLUDE_DIRECTORIES( 
    .
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/roboy/datastructures
    ${CMAKE_SOURCE_DIR}/src/roboy/utils
		${CMAKE_SOURCE_DIR}/src/roboy/robot
    ${CMAKE_SOURCE_DIR}/src/roboy/vision
    ${CMAKE_SOURCE_DIR}/src/roboy/starmind
    ${CMAKE_SOURCE_DIR}/src/roboy/bodyactivities
    ${CMAKE_SOURCE_DIR}/src/roboy/behaviours
    ${CMAKE_CURRENT_BINARY_DIR}
    ${LIB_OPENCV_INCLUDE_DIR}
  	${LIB_HIGHGUI_INCLUDE_DIRS}
  	${Boost_INCLUDE_DIRS}
  	${ALUT_INCLUDE_DIR}
  	${MVBLUEFOX_INCLUDE_DIR}
	
)

#################################################
#        LIBS                                   #
################################################
SET( LIB ${LIB}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
	${ALUT_LIBRARY}
	${MVBLUEFOX_LIBRARIES}
    ${CMAKE_BINARY_DIR}/src/roboy/datastructures/libRoboyDataStructures.so
	${CMAKE_BINARY_DIR}/src/roboy/utils/libRoboyUtils.so
  ${CMAKE_BINARY_DIR}/src/roboy/robot/libRoboyRobot.so
	${CMAKE_BINARY_DIR}/src/roboy/vision/libRoboyVision.so
	${CMAKE_BINARY_DIR}/src/roboy/starmind/libRoboyStarmindConnector.so
	${CMAKE_BINARY_DIR}/src/roboy/bodyactivities/libRoboyBodyActivities.so
	${CMAKE_BINARY_DIR}/src/roboy/behaviours/libRoboyBehaviours.so

)

MESSAGE(STATUS " Boost: "  ${Boost_THREAD_LIBRARY} 	${Boost_INCLUDE_DIRS})
 

################################################
#   All component sources must be named here   #
################################################
SET( PROJECT_SRCS
  RoboyState.cpp
  RoboyGUI.cpp
  RoboyStateMachineMain.cpp
)

MESSAGE(STATUS "COMPILER_FLAGS " ${COMPILER_FLAGS})

ADD_EXECUTABLE( ${PROJECT_NAME} ${PROJECT_SRCS})
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES COMPILER_FLAGS CXX_FLAGS )
TARGET_LINK_LIBRARIES( ${PROJECT_NAME} ${LIB} )

 #ADD_LIBRARY( ${PROJECT_NAME} SHARED ${PROJECT_SRCS} )


#SET_TARGET_PROPERTIES(
#	${PROJECT_NAME}
#	PROPERTIES
#	DEBUG_POSTFIX d
#)

#TARGET_LINK_LIBRARIES( ${COMP_NAME}Comp ${COMP_NAME} )

ADD_SUBDIRECTORY( ./introduction )
#sADD_SUBDIRECTORY( ./visioninteraction )
ADD_SUBDIRECTORY( ./testLegSwing )
ADD_SUBDIRECTORY( ./theatre )
ADD_SUBDIRECTORY( ./testFacial )

