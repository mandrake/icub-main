# Copyright: 2010 RobotCub Consortium
# Author: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

SET(PROJECTNAME velocityObserver)

PROJECT(${PROJECTNAME})

SET(folder_source main.cpp)

SOURCE_GROUP("Source Files" FILES ${folder_source})

INCLUDE_DIRECTORIES(${ctrlLib_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ctrlLib
                                     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)



