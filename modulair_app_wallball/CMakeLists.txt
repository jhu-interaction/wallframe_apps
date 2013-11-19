cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

find_package(Qt4 REQUIRED)
if(QT_FOUND)
  set (QT_USE_QTWEBKIT TRUE)
  set (QT_USE_QTOPENGL TRUE)
  include(${QT_USE_FILE})
  add_definitions(${QT_DEFINITIONS})
  add_definitions(-DQT_NO_KEYWORDS)
else(QT_FOUND)
  message(FATAL: QT NOT FOUND)
endif(QT_FOUND)

## Provided Box2D and IRRLICHT cmake finders
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

find_package(Box2D REQUIRED)
if (BOX2D_FOUND)
  include_directories(${BOX2D_INCLUDE_DIRS})
#  set(LIBS ${LIBS} ${IRRLICHT_LIBRARIES})
else (BOX2D_FOUND)
  message (SEND_ERROR "${CMAKE_CURRENT_SOURCE_DIR} requires Box2d")
endif (BOX2D_FOUND)

find_package(Irrlicht REQUIRED)
if (IRRLICHT_FOUND)
  include_directories(${IRRLICHT_INCLUDE_DIRS})
#  set(LIBS ${LIBS} ${IRRLICHT_LIBRARIES})
else (IRRLICHT_FOUND)
  message (SEND_ERROR "${CMAKE_CURRENT_SOURCE_DIR} requires Irrlicht")
endif (IRRLICHT_FOUND)

## Set Paths
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

## Qt MOC Generator (for Q_OBJECT behaviors)
set(_moc ${CMAKE_CURRENT_BINARY_DIR}/${_current_MOC})

set(WALLBALL_APP_SOURCES 
  src/WallBall.cpp
  src/WallBallWidget.cpp
  src/Components/CController.cpp
  src/Components/CGraphicsObject.cpp
  src/Components/CKeyboardController.cpp
  src/Components/CKinectController.cpp
  src/Components/CPhysicsObject.cpp
  src/Components/CSpawnPoint.cpp
  src/GameObject/Component.cpp
  src/GameObject/GameObject.cpp
  src/GameObject/GameObjectManager.cpp
  src/GameObject/GOBall.cpp
  src/GameObject/GOScore.cpp
  src/GameObject/GOBlock.cpp
  src/GameObject/GOPortal.cpp
  src/GameObject/GOSpawnPoint.cpp
  src/GameObject/GOStar.cpp
  src/GameObject/LevelObjects.cpp
  src/Managers/GraphicsManager.cpp
  src/Managers/InputManager.cpp
  src/Managers/PhysicsManager.cpp
  )

set(WALLBALL_APP_HEADERS 
  include/WallBallWidget.h
  include/Components/CController.h
  include/Components/CGraphicsObject.h
  include/Components/CKeyboardController.h
  include/Components/CKinectController.h
  include/Components/CPhysicsObject.h
  include/Components/CSpawnPoint.h
  include/GameObject/Component.h
  include/GameObject/GameObject.h
  include/GameObject/GameObjectManager.h
  include/GameObject/GOBall.h
  include/GameObject/GOScore.h
  include/GameObject/GOBlock.h
  include/GameObject/GOPortal.h
  include/GameObject/GOSpawnPoint.h
  include/GameObject/GOStar.h
  include/GameObject/LevelObjects.h
  include/Managers/GraphicsManager.h
  include/Managers/InputManager.h
  include/Managers/PhysicsManager.h
  )

## Create MOC wrappers for headers
qt4_wrap_cpp(WALLBALL_APP_HEADERS_MOC ${WALLBALL_APP_HEADERS})

rosbuild_add_executable(wallball_app 	${WALLBALL_APP_SOURCES})
target_link_libraries(wallball_app 	${QT_LIBRARIES}
                                        ${IRRLICHT_LIBRARIES}
                                        ${WALLBALL_APP_LIBRARIES}
                                        ${BOX2D_LIBRARIES})
