#
# artoolkitX Camera Calibration (Desktop) Utility
#
# To invoke CMake on this file, from the chosen build directory, invoke:
#     cmake <path to directory containing this file>.
#
# Author(s): Philip Lamb <phil@artoolkitx.org>
#

#
# Packages required: artoolkitx-dev libjpeg-dev libopencv-calib3d-dev libssl-dev libcurl4-openssl-dev libconfig-dev
#

cmake_minimum_required( VERSION 3.2 )

# Any directives that must go before the project command.

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Specifies what build types (configurations) will be available." FORCE)
if(DEFINED CMAKE_BUILD_TYPE)
  set_property( CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${CMAKE_CONFIGURATION_TYPES} )
endif()

project("artoolkitx_calib_camera"
        VERSION 1.0.0
        LANGUAGES CXX C
)

set(ARTOOLKITX_HOME "${CMAKE_CURRENT_SOURCE_DIR}/../../artoolkitx")
set(ARTOOLKITX_CAMERA_CALIBRATION_HOME "${CMAKE_CURRENT_SOURCE_DIR}/..")

add_definitions("-DARX_TARGET_PLATFORM_LINUX=1")

if (${CMAKE_BUILD_TYPE} MATCHES "Debug")
    add_definitions("-DDEBUG=1")
endif()

# Search prefix.
list(APPEND CMAKE_MODULE_PATH "${ARTOOLKITX_HOME}/Source/cmake")
if(ARX_TARGET_PLATFORM_VARIANT)
    list(APPEND CMAKE_FIND_ROOT_PATH "${ARTOOLKITX_HOME}/Source/depends/linux-${ARX_TARGET_PLATFORM_VARIANT}")
    list(APPEND CMAKE_FIND_ROOT_PATH "${ARTOOLKITX_CAMERA_CALIBRATION_HOME}/depends/linux-${ARX_TARGET_PLATFORM_VARIANT}")
else()
    list(APPEND CMAKE_FIND_ROOT_PATH "${ARTOOLKITX_HOME}/Source/depends/linux")
    list(APPEND CMAKE_FIND_ROOT_PATH "${ARTOOLKITX_CAMERA_CALIBRATION_HOME}/depends/linux")
endif()
if ("${ARX_TARGET_PLATFORM_VARIANT}" STREQUAL "raspbian")
    list(APPEND CMAKE_FIND_ROOT_PATH "/opt/vc/")
    set(GL_PREFER_EMBEDDED 1)
endif()

# Installation settings.
# If the user didn't define CMAKE_INSTALL_PREFIX, set a local install destination.
# When building packages, this will typically be overridden with DESTDIR (e.g. "make install DESTDIR=/tmp").
if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set (CMAKE_INSTALL_PREFIX "${ARTOOLKITX_CAMERA_CALIBRATION_HOME}" CACHE PATH "default install path" FORCE )
endif()

# Enable C++11 for GCC/Clang compilers.
set(CMAKE_CXX_STANDARD "11")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(${ARTOOLKITX_CAMERA_CALIBRATION_HOME})

find_package(SDL2 REQUIRED)
string(STRIP "${SDL2_LIBRARY}" SDL2_LIBRARY)
include_directories(${SDL2_INCLUDE_DIR})
if ("${ARX_TARGET_PLATFORM_VARIANT}" STREQUAL "raspbian")
    list(APPEND SDL2_LIBRARY -Wl,--no-undefined -lm -L/opt/vc/lib -lbcm_host -ldl -ldl -lpthread -lrt)
endif()

find_package(OpenGL)
find_package(OpenGLES2)
find_package(OpenGL3)
if ((OPENGL_FOUND OR OpenGL3_FOUND) AND OPENGLES2_FOUND)
    if(GL_PREFER_EMBEDDED)
        set(OPENGL_FOUND 0)
        set(OpenGL3_FOUND 0)
        message(STATUS "Opting for embedded OpenGL")
    else()
        set(OPENGLES2_FOUND 0)
        message(STATUS "Opting for desktop OpenGL. Set GL_PREFER_EMBEDDED to change.")
    endif()
endif()

find_package(JPEG REQUIRED)
include_directories(${JPEG_INCLUDE_DIR})

find_path(
    OPENCV_INCLUDE_DIR
    NAMES calib3d.hpp
    PATH_SUFFIXES opencv2/calib3d
)
include_directories(${OPENCV_INCLUDE_DIR}/../..)
find_library(OPENCV_CALIB3D_LIBRARY NAMES opencv_calib3d)
find_library(OPENCV_FEATURES2D_LIBRARY NAMES opencv_features2d)
find_library(OPENCV_IMGPROC_LIBRARY NAMES opencv_imgproc)
find_library(OPENCV_FLANN_LIBRARY NAMES opencv_flann)
find_library(OPENCV_CORE_LIBRARY NAMES opencv_core)

find_package(CURL REQUIRED)
include_directories(${CURL_INCLUDE_DIRS})

find_package(OpenSSL REQUIRED)
include_directories(${CURL_INCLUDE_DIRS})

find_package(PkgConfig)
pkg_check_modules(LIBCONFIG REQUIRED libconfig)
include_directories(${LIBCONFIG_INCLUDE_DIRS})

include_directories(${ARTOOLKITX_HOME}/Source/depends/linux/include)

include(${ARTOOLKITX_HOME}/SDK/lib/ARX/ARX.cmake)

set(SOURCE
    ../calib_camera.cpp
    ../calib_camera.h
    ../Calibration.hpp
    ../Calibration.cpp
    ../calc.cpp
    ../calc.hpp
    ../fileUploader.c
    ../fileUploader.h
    ../flow.cpp
    ../flow.hpp
    ../prefs.hpp
    ../prefsLibConfig.cpp
    ../prefsNull.cpp
    ../Eden/Eden.h
    ../Eden/EdenError.h
    ../Eden/EdenGLFont.c
    ../Eden/EdenGLFont.h
    ../Eden/EdenMath.h
    ../Eden/EdenMessage.c
    ../Eden/EdenMessage.h
    ../Eden/EdenSurfaces.c
    ../Eden/EdenSurfaces.h
    ../Eden/EdenTime.c
    ../Eden/EdenTime.h
    ../Eden/EdenUtil.c
    ../Eden/EdenUtil.h
    ../Eden/EdenUIInput.c
    ../Eden/EdenUIInput.h
    ../Eden/glStateCache.c
    ../Eden/glStateCache.h
    ../Eden/gluttext.h
    ../Eden/readtex.c
    ../Eden/readtex.h
    ../Eden/gluttext/glut_8x13.c
    ../Eden/gluttext/glut_9x15.c
    ../Eden/gluttext/glut_bitmap.c
    ../Eden/gluttext/glut_bwidth.c
    ../Eden/gluttext/glut_hel10.c
    ../Eden/gluttext/glut_hel12.c
    ../Eden/gluttext/glut_hel18.c
    ../Eden/gluttext/glut_mroman.c
    ../Eden/gluttext/glut_roman.c
    ../Eden/gluttext/glut_stroke.c
    ../Eden/gluttext/glut_swidth.c
    ../Eden/gluttext/glut_tr10.c
    ../Eden/gluttext/glut_tr24.c
    ../Eden/gluttext/glutbitmap.h
    ../Eden/gluttext/glutstroke.h
)

add_executable(${CMAKE_PROJECT_NAME} ${SOURCE})

add_dependencies(${CMAKE_PROJECT_NAME}
    ARX
)

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    INSTALL_RPATH "\$ORIGIN"
)

target_link_libraries(${CMAKE_PROJECT_NAME}
    ARX
    ${SDL2_LIBRARY}
    ${JPEG_LIBRARIES}
    ${OPENCV_CALIB3D_LIBRARY} ${OPENCV_FEATURES2D_LIBRARY} ${OPENCV_IMGPROC_LIBRARY} ${OPENCV_FLANN_LIBRARY} ${OPENCV_CORE_LIBRARY}
    ${CURL_LIBRARIES} ${OPENSSL_LIBRARIES}
    ${LIBCONFIG_LIBRARIES}
    pthread
    m
)

if(OPENGL_FOUND)
    target_include_directories(${PROJECT_NAME} PRIVATE ${OPENGL_INCLUDE_DIR})
    target_link_libraries(${CMAKE_PROJECT_NAME} ${OPENGL_LIBRARIES})
endif()
if(OPENGLES2_FOUND)
    target_include_directories(${PROJECT_NAME} PRIVATE ${OPENGLES2_INCLUDE_DIR})
    target_link_libraries(${CMAKE_PROJECT_NAME} ${OPENGLES2_LIBRARIES})
endif()
if(OpenGL3_FOUND)
    target_include_directories(${PROJECT_NAME} PRIVATE ${OpenGL3_INCLUDE_PATH})
    target_link_libraries(${CMAKE_PROJECT_NAME} ${OpenGL3_LIBRARIES})
endif()

get_directory_property(ARXCC_DEFINES DIRECTORY ${CMAKE_SOURCE_DIR} COMPILE_DEFINITIONS)
foreach(d ${ARXCC_DEFINES})
    message(STATUS "Defined: " ${d})
endforeach()

install(TARGETS ${CMAKE_PROJECT_NAME}
    RUNTIME DESTINATION .
)

