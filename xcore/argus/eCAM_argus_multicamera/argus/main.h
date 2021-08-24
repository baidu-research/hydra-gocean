/*
 * Copyright (c) 2016-2018, e-con Systems India Pvt. Ltd CORPORATION. All rights reserved.
 * Author : Waiss Kharni <waisskharni.sm@e-consystems.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef MAIN_H
#define MAIN_H

#include "ProcessFrameThread.h"
#include "MasterSlaveConfig.h"
#include "CameraModuleEGL.h"
#include "ControlSettings.h"
#include "CommonOptions.h"
#include "VersionConfig.h"
#include "UserInterface.h"
#include "VideoEncoder.h"
#include "Thread.h"
#include "Error.h"
#include "Util.h"

#include <Argus/Argus.h>
#include <nvbuf_utils.h>
#include <NvEglRenderer.h>
#include <EGLStream/EGLStream.h>
#include <Argus/Ext/BayerAverageMap.h>
#include <Argus/Ext/DolWdrSensorMode.h>

#define YELLOW  "\x1B[33m"
#define WHITE  "\x1B[37m"
#define GREEN  "\x1B[32m"
#define RESET "\x1B[0m"
#define BLUE  "\x1B[34m"
#define RED  "\x1B[31m"

#include <sys/mman.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <limits>
#include <math.h>

extern uint32_t	g_deviceID;
extern uint32_t g_masterDev;
extern uint32_t	g_streamState;
extern uint32_t	g_captureType;
extern uint32_t	g_controlState;

extern float g_masterFrameRate;

extern bool	g_calibrate;
extern bool	g_captureState;
extern bool	g_videoRecordFlag;
extern bool	g_videoStopRecord;
extern bool	g_burstRecordFlag;
extern bool	g_frameRateChange;
extern bool	g_burstStopRecord;
extern bool	g_burstCaptureFlag;
extern bool	g_getStreamMetadata;

enum {
    CAPTURE_TYPE_JPEG = 1,
    CAPTURE_TYPE_RAW,
    CAPTURE_TYPE_BAYER,
    CAPTURE_TYPE_H264,
};

enum {
    CTRL_STATE_GET = 1,
    CTRL_STATE_SET,
};

enum {
    LIGHTSRC_60HZ = 0,
    LIGHTSRC_50HZ,
};

static const uint32_t           MAX_CAMERA_NUM = 6;
static const uint32_t           MAX_SYNC_CAMERAS = 4;
static const float		DEFAULT_FPS = 30.0f;
static const float		MIN_FPS = 2.5f;
static const uint32_t		MASTER_DEVNODE = 2;
void signal_handler(int sig_num);

namespace ArgusSamples
{

// Globals and derived constants.
extern UniqueObj<CameraProvider> 	g_cameraProvider;
extern std::vector<CameraDevice*> 	g_cameraDevices;

// Debug print macros.
#define PRODUCER_PRINT(...) printf("PRODUCER: " __VA_ARGS__)
#define CONSUMER_PRINT(...) printf("CONSUMER: " __VA_ARGS__)

}; //NameSpace ArgusSamples

#endif //MAIN_H
