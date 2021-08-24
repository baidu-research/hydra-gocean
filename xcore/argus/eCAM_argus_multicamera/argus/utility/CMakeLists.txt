# Copyright (c) 2016-2018, e-con Systems India Pvt. Ltd. All rights reserved.
# Author - Waiss Kharni <waisskharni.sm@e-consystems.com>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of NVIDIA CORPORATION nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required (VERSION 2.6)

project(eCAM_argusutility)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" "${CMAKE_MODULE_PATH}")

find_package(Argus COMPONENTS ARGUS_INCLUDE_DIR OPTIONAL_COMPONENTS ARGUS_LIBRARY)
find_package(OpenGLES)
find_package(EGL REQUIRED)
find_package(X11 REQUIRED)
find_package(NVMMAPI)

find_package(PkgConfig REQUIRED)
pkg_check_modules(Cairo REQUIRED cairo)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)
pkg_check_modules(GLIB REQUIRED glib-2.0)

set(SOURCES
    ArgusHelpers.cpp
    CameraModuleEGL.cpp
    CommonOptions.cpp
    ControlSettings.cpp
    MasterSlaveConfig.cpp
    Observed.cpp
    Options.cpp
    ProcessFrameThread.cpp
    Thread.cpp
    UserInterface.cpp
    Util.cpp
    VideoEncoder.cpp
    )

if(NVMMAPI_FOUND AND OPENGLES_FOUND)
    set(SOURCES
        ${SOURCES}
	nvmmapi/NvEglRenderer.cpp
	nvmmapi/NvElement.cpp
	nvmmapi/NvElementProfiler.cpp
	nvmmapi/NvLogging.cpp
	nvmmapi/NvV4l2ElementPlane.cpp
	nvmmapi/NvVideoEncoder.cpp
	nvmmapi/NvV4l2Element.cpp
	nvmmapi/NvBuffer.cpp
        )
    include_directories(
        ${NVMMAPI_INCLUDE_DIR}
        )
    add_definitions(
        ${NVMMAPI_DEFINITIONS}
        )
endif(NVMMAPI_FOUND AND OPENGLES_FOUND)

include_directories(
    ${ARGUS_INCLUDE_DIR}
    ${EGL_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${GSTREAMER_INCLUDE_DIRS}
    ${GLIB_INCLUDE_DIRS}
    ${OPENGLES_INCLUDE_DIR}
    )

add_library(${PROJECT_NAME} STATIC ${SOURCES})

target_link_libraries(${PROJECT_NAME}
    pthread
    v4l2
    ${X11_LIBRARIES}
    ${EGL_LIBRARIES}
    ${GSTREAMER_LIBRARIES}
    ${GLIB_LIBRARIES}
    )

if(OPENGLES_FOUND)
    target_link_libraries(${PROJECT_NAME}
        ${OPENGLES_LIBRARIES}
        )
endif(OPENGLES_FOUND)

if(NVMMAPI_FOUND)
    target_link_libraries(${PROJECT_NAME}
        ${NVMMAPI_LIBRARIES}
        )
endif(NVMMAPI_FOUND)
