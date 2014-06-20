cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

### OpenGL
FIND_PACKAGE( OpenGL REQUIRED )

### Bring in the Qt Libraries ###
find_package(Qt4 REQUIRED QtCore QtGui QtOpenGL)
if(QT_FOUND)
    set (QT_USE_QTWEBKIT TRUE)
    set (QT_USE_QTOPENGL TRUE)
    include(${QT_USE_FILE})
    add_definitions(${QT_DEFINITIONS})
    add_definitions(-DQT_NO_KEYWORDS)
else(QT_FOUND)
    message(FATAL: QT NOT FOUND)
endif(QT_FOUND)

##### OGRE and BOOST
if(UNIX)
    if(EXISTS "/usr/local/lib/OGRE/cmake")

      set(CMAKE_MODULE_PATH "/usr/local/lib/OGRE/cmake/;${CMAKE_MODULE_PATH}")
      set(OGRE_SAMPLES_INCLUDEPATH "/usr/local/share/OGRE/samples/Common/include/") # We could just *assume* that developers uses this basepath : /usr/local

    elseif(EXISTS "/usr/lib/OGRE/cmake")

      set(CMAKE_MODULE_PATH "/usr/lib/OGRE/cmake/;${CMAKE_MODULE_PATH}")
      set(OGRE_SAMPLES_INCLUDEPATH "/usr/share/OGRE/samples/Common/include/") # Otherwise, this one

        else()
      message(SEND_ERROR "Failed to find module path.")
        endif(EXISTS "/usr/local/lib/OGRE/cmake")
endif(UNIX)

if (CMAKE_BUILD_TYPE STREQUAL "")
  # CMake defaults to leaving CMAKE_BUILD_TYPE empty. This screws up
  # differentiation between debug and release builds.
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build, options are: None (CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif ()
 
set(CMAKE_DEBUG_POSTFIX "_d")
 
set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/dist")
 
find_package(OGRE REQUIRED)

if (NOT OGRE_BUILD_PLATFORM_IPHONE)
    if (WIN32 OR APPLE)
        set(Boost_USE_STATIC_LIBS TRUE)
    else ()
        # Statically linking boost to a dynamic Ogre build doesn't work on Linux 64bit
        set(Boost_USE_STATIC_LIBS ${OGRE_STATIC})
    endif ()
    if (MINGW)
        # this is probably a bug in CMake: the boost find module tries to look for
        # boost libraries with name libboost_*, but CMake already prefixes library
        # search names with "lib". This is the workaround.
        set(CMAKE_FIND_LIBRARY_PREFIXES ${CMAKE_FIND_LIBRARY_PREFIXES} "")
    endif ()
    set(Boost_ADDITIONAL_VERSIONS "1.44" "1.44.0" "1.42" "1.42.0" "1.41.0" "1.41" "1.40.0" "1.40" "1.39.0" "1.39" "1.38.0" "1.38" "1.37.0" "1.37" )
    # Components that need linking (NB does not include header-only components like bind)
        set(OGRE_BOOST_COMPONENTS thread date_time system filesystem)
    find_package(Boost COMPONENTS ${OGRE_BOOST_COMPONENTS} QUIET)
    if (NOT Boost_FOUND)
        # Try again with the other type of libs
        set(Boost_USE_STATIC_LIBS NOT ${Boost_USE_STATIC_LIBS})
        find_package(Boost COMPONENTS ${OGRE_BOOST_COMPONENTS} QUIET)
    endif()

    # Set up referencing of Boost
    include_directories(${Boost_INCLUDE_DIR})
    add_definitions(-DBOOST_ALL_NO_LIB)
    set(OGRE_LIBRARIES ${OGRE_LIBRARIES} ${Boost_LIBRARIES})
endif()

## Set Paths
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


## Qt MOC Generator (for Q_OBJECT behaviors)
set(_moc ${CMAKE_CURRENT_BINARY_DIR}/${_current_MOC})

## Set your source files here (anything thats has a Q_OBJECT in it)
set(CPP_EXAMPLE_APP_SOURCES 
    src/museum_app.cpp
    src/ogrewidget.cpp
    src/MuseumApp.cpp
    src/WPainting.cpp
    src/Utils.cpp        
    src/Room.cpp
    )

set(CPP_EXAMPLE_APP_HEADERS 
    include/museum_app.h
    include/ogrewidget.h
    include/MuseumApp.h
    include/WPainting.h
    include/Utils.h
    include/Room.h
    include/CameraMan.h
    )

## Create MOC wrappers for headers
qt4_wrap_cpp(CPP_EXAMPLE_APP_HEADERS_MOC ${CPP_EXAMPLE_APP_HEADERS})

rosbuild_add_library(${PROJECT_NAME}	${CPP_EXAMPLE_APP_HEADERS}
                                        ${CPP_EXAMPLE_APP_SOURCES}
                                        ${CPP_EXAMPLE_APP_HEADERS_MOC})


rosbuild_add_boost_directories()
rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_boost_directories()

rosbuild_add_executable(museum_app 	${CPP_EXAMPLE_APP_SOURCES})


set_target_properties(museum_app PROPERTIES DEBUG_POSTFIX _d)

target_link_libraries(museum_app 		${PROJECT_NAME}
                                            ${QT_LIBRARIES}
                                            ${OGRE_LIBRARIES}
                                            ${OPENGL_LIBRARIES})

# file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/dist/bin)
# file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/dist/media)

# set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/dist/bin)

# if(UNIX)

# 	install(TARGETS OgreApp
# 		RUNTIME DESTINATION bin
# 		CONFIGURATIONS All)

# 	install(DIRECTORY ${CMAKE_SOURCE_DIR}/dist/media
# 		DESTINATION ./
# 		CONFIGURATIONS Release RelWithDebInfo Debug
# 	)

# 	install(FILES ${CMAKE_SOURCE_DIR}/dist/bin/plugins.cfg
# 		${CMAKE_SOURCE_DIR}/dist/bin/resources.cfg
# 		DESTINATION bin
# 		CONFIGURATIONS Release RelWithDebInfo Debug
# 	)

# endif(UNIX)
