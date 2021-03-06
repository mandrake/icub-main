# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

add_subdirectory(controlBoardDumper)
add_subdirectory(simpleClient)
add_subdirectory(testStereoMatch)
add_subdirectory(fingersTuner)
add_subdirectory(imuFilter)
add_subdirectory(embObjProtoTools/boardTransceiver)

#canLoader needs GtkPlus, but it contains a library needed by other modules
add_subdirectory(canLoader)
add_subdirectory(ethLoader)


# Tools available both for Qt5 and GTK2
if(ICUB_USE_Qt5)
   add_subdirectory(iCubSkinGui-qt)
   add_subdirectory(frameGrabberGui2-qt)
   add_subdirectory(skinManagerGui-qt)
endif()
if(ICUB_USE_GTK2)
   add_subdirectory(iCubSkinGui-gtk)
   add_subdirectory(frameGrabberGui2-gtk)
   if(ICUB_GTK2_LEGACY)
      message(STATUS "GTK2 legacy detected, skipping skinManagerGui-gtk")
   else()
      add_subdirectory(skinManagerGui-gtk)
   endif()
endif()
if(NOT ICUB_USE_Qt5 AND NOT ICUB_USE_GTK2)
   message(STATUS "Qt5 and GTK2 not found/selected, skipping iCubSkinGui")
   message(STATUS "Qt5 and GTK2 not found/selected, skipping frameGrabberGui2")
   message(STATUS "Qt5 and GTK2 not found/selected, skipping skinManagerGui")
endif()


if(ICUB_USE_GTK2)
   add_subdirectory(robotMotorGui)
else(ICUB_USE_GTK2)
   message(STATUS "GTK2 not found/selected, skipping robotMotorGui")
endif(ICUB_USE_GTK2)

if(ICUB_USE_SDL)
   add_subdirectory(joystickCtrl)
   add_subdirectory(joystickCheck)
else(ICUB_USE_SDL)
   message(STATUS "SDL not found/selected, skipping joystickCtrl")
endif(ICUB_USE_SDL)

if(ICUB_USE_GTK2)
   add_subdirectory(focusTool)
   if(ICUB_USE_INTERFACEGUI)
     add_subdirectory(iCubInterfaceGui)
   endif(ICUB_USE_INTERFACEGUI)
else(ICUB_USE_GTK2)
   message(STATUS "GTK2 not found/selected, skipping focusTool")
   message(STATUS "GTK2 not found/selected, skipping iCubInterfaceGui")
endif(ICUB_USE_GTK2)

if(ICUB_USE_Qt5 AND ICUB_USE_OpenGL AND ICUB_USE_GLUT)
    add_subdirectory(iCubGui)
else()
    message(STATUS "Qt5 or OpenGL or GLUT not found/selected, skipping iCubGui")
endif()

if(ICUB_USE_OpenCV)
    if(ICUB_OpenCV_LEGACY)
        message(STATUS "OpenCV legacy detected, skipping stereoCalib")
        # put hereafter modules that rely on old versions of OpenCV declared as legacy
    else(ICUB_OpenCV_LEGACY)
        add_subdirectory(stereoCalib)
    endif(ICUB_OpenCV_LEGACY)
else(ICUB_USE_OpenCV)
    message(STATUS "OpenCV not found/selected, skipping stereoCalib")
endif(ICUB_USE_OpenCV)

if((ICUB_USE_GSL AND ICUB_USE_IPOPT) AND (ICUB_USE_OpenCV AND (NOT ICUB_OpenCV_LEGACY)))
    add_subdirectory(depth2kin)
else()
    message(STATUS "Unmatched dependencies, skipping depth2kin")
endif()
