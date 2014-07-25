CMAKE_MINIMUM_REQUIRED (VERSION 2.6) 

set(CMAKE_DISABLE_SOURCE_CHANGES ON)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

SET(CMAKE_MODULE_PATH
    ${CMAKE_SOURCE_DIR}/modules
)

PROJECT( Roboy )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -ggdb")
SET(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")

MESSAGE(STATUS " C LINK LIB FLAGS: " ${CMAKE_C_LINK_FLAGS})
MESSAGE(STATUS " SHARED LIB FLAGS: " ${CMAKE_SHARED_LIBRARY_LINK_C_FLAGS}) 
MESSAGE(STATUS " CMAKE FLAGS: " ${CMAKE_CXX_FLAGS})
# -std=c++0x -pthread")

#################################################
#        GLOBAL SET COMMANDS                    #
#################################################
#IF (${CMAKE_BUILD_TYPE} STREQUAL "Release")
#    SET(CMAKE_VERBOSE_MAKEFILE OFF)
#ELSEIF (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
#   SET(CMAKE_VERBOSE_MAKEFILE ON)
#ENDIF (${CMAKE_BUILD_TYPE} STREQUAL "Release")

#SET(CMAKE_MODULE_PATH
#    ${CMAKE_SOURCE_DIR}/modules
#)


#################################################
#        We always want to be able to include   #
#        util headers directly                  #
#################################################
INCLUDE_DIRECTORIES(
    src/roboy/utils
)

#################################################
#        BUILD SUBDIRECTORIES                   #
#################################################


ADD_SUBDIRECTORY( src/roboy/datastructures )
ADD_SUBDIRECTORY( src/roboy/utils )
ADD_SUBDIRECTORY( src/roboy/vision )
ADD_SUBDIRECTORY( src/roboy/starmind )
ADD_SUBDIRECTORY( src/roboy/robot )
ADD_SUBDIRECTORY( src/roboy/bodyactivities )
ADD_SUBDIRECTORY( src/roboy/behaviours )
ADD_SUBDIRECTORY( src/roboy/main )

# FIXME: these should also have cmake file in each directory
ADD_SUBDIRECTORY( src/roboy/tools/RoboyEyePicker )
ADD_SUBDIRECTORY( src/roboy/tools/RoboyFaceTrainer )
ADD_SUBDIRECTORY( src/roboy/tools/RoboyVideoCapture )
ADD_SUBDIRECTORY( src/roboy/tools/RoboyTensionControl )
ADD_SUBDIRECTORY( src/roboy/tools/RoboyInitialization )
ADD_SUBDIRECTORY( src/roboy/tools/RoboySingleMotorControl )
ADD_SUBDIRECTORY( src/roboy/tools/RoboyModifyTrajectories )
ADD_SUBDIRECTORY( src/roboy/tools/RoboyPlayback )
ADD_SUBDIRECTORY( src/roboy/tests/RoboyFacialExpressionTester )
ADD_SUBDIRECTORY( src/roboy/tests/RoboyFileParserTester )
ADD_SUBDIRECTORY( src/roboy/tests/RoboyMVBlueFoxTester )
ADD_SUBDIRECTORY( src/roboy/tests/RoboyLegSwing )
ADD_SUBDIRECTORY( src/roboy/tests/RoboyLegLift )
ADD_SUBDIRECTORY( src/roboy/tests/RoboyTestSequence )
ADD_SUBDIRECTORY( src/roboy/tests/RoboyTestSensors )
ADD_SUBDIRECTORY( src/roboy/tests/RoboyTestNeckMotion )
ADD_SUBDIRECTORY( src/roboy/tests/RoboyTestNeckMotionAll )
ADD_SUBDIRECTORY( src/roboy/tests/RoboyDemo )

#FIND_PACKAGE( OpenCV REQUIRED )
#FIND_PACKAGE( Festival REQUIRED )
#FIND_PACKAGE( EST REQUIRED )
#FIND_PACKAGE( ALUT REQUIRED )

#MESSAGE (STATUS "OPENCV: " ${OpenCV_LIBS})



#MESSAGE(STATUS "FESTIVAL " ${FESTIVAL_INCLUDE_DIR})
#MESSAGE(STATUS "EST " ${EST_INCLUDE_DIR})

#ADD_EXECUTABLE( helloworld helloworld.cxx )
#ADD_EXECUTABLE( facerec_video facerec_video.cpp )
#ADD_EXECUTABLE( peopledetect peopledetect.cpp )
#ADD_EXECUTABLE( RoboyFaceDetectionVideo RoboyFaceDetectionVideo.cpp )
#ADD_EXECUTABLE( RoboyEyePicker RoboyEyePicker.cpp )
#ADD_EXECUTABLE( FestivalHelloWorld FestivalHelloWorld.cpp )
#ADD_EXECUTABLE( soundtest soundtest.cpp )

#TARGET_LINK_LIBRARIES( helloworld ${OpenCV_LIBS} )
#TARGET_LINK_LIBRARIES( peopledetect ${OpenCV_LIBS} )
#TARGET_LINK_LIBRARIES( facerec_video ${OpenCV_LIBS} )
#TARGET_LINK_LIBRARIES( RoboyFaceDetectionVideo ${OpenCV_LIBS} )
##TARGET_LINK_LIBRARIES( RoboyEyePicker ${OpenCV_LIBS} )
#TARGET_LINK_LIBRARIES( FestivalHelloWorld ${FESTIVAL_LIBRARY} ${EST_LIBRARIES} )
#TARGET_LINK_LIBRARIES( soundtest ${ALUT_LIBRARY} )

