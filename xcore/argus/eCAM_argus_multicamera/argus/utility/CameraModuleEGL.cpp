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
#include "../main.h"
#include "CameraModuleEGL.h"

namespace ArgusSamples
{
CameraModulesEGL::CameraModulesEGL(CaptureSession *session):
	  m_captureSession(session)
	, m_initialized(false)
{
}

bool CameraModulesEGL::initialize(CameraDevice *device, Argus::SensorMode *sensormode)
{
    if (m_initialized)
	return true;

    m_sensorModeID = sensormode;
    m_camerDevice  = device;

    // Create the capture session using the first device and get the core interface.
    ICaptureSession *iCaptureSession = interface_cast<ICaptureSession>(m_captureSession);
    if (!iCaptureSession)
        ORIGINATE_ERROR("Failed to create CaptureSession");

    ISensorMode *iSensorMode = interface_cast<ISensorMode>(m_sensorModeID);
    if (!iSensorMode)
	ORIGINATE_ERROR("Failed to get ISensorMode interface");

    // Create the OutputStream.
    UniqueObj<OutputStreamSettings> streamSettings(
        iCaptureSession->createOutputStreamSettings(STREAM_TYPE_EGL));
    IEGLOutputStreamSettings *iEglStreamSettings =
        interface_cast<IEGLOutputStreamSettings>(streamSettings);
    if (!iEglStreamSettings)
        ORIGINATE_ERROR("Failed to create EglOutputStreamSettings");

    iEglStreamSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
    iEglStreamSettings->setMode(EGL_STREAM_MODE_MAILBOX);
    iEglStreamSettings->setResolution(iSensorMode->getResolution());
    iEglStreamSettings->setMetadataEnable(true);

    m_outputStream.reset(iCaptureSession->createOutputStream(streamSettings.get()));

    // Create capture request and enable the output stream.
    m_request.reset(iCaptureSession->createRequest());
    IRequest *iRequest = interface_cast<IRequest>(m_request);
    if (!iRequest)
        ORIGINATE_ERROR("Failed to create Request");

    iRequest->enableOutputStream(m_outputStream.get());

    PROPAGATE_ERROR(setStreamProperties());

    m_initialized = true;
    return true;
}

bool CameraModulesEGL::setStreamProperties()
{
    ICaptureSession *iSession = interface_cast<ICaptureSession>(m_captureSession);
    ISensorMode *iSensorMode = interface_cast<ISensorMode>(m_sensorModeID);
    IRequest *iRequest = interface_cast<IRequest>(m_request);
    if (!iSensorMode || !iSession || !iRequest)
	ORIGINATE_ERROR("Failed to get Stream Interfaces");

    IAutoControlSettings *iAutoControlSettings = interface_cast<IAutoControlSettings>(
					iRequest->getAutoControlSettings());

    ISourceSettings *iSourceSettings = interface_cast<ISourceSettings>(
					iRequest->getSourceSettings());

    if (!iAutoControlSettings || !iSourceSettings)
	ORIGINATE_ERROR("Failed to get Control Settings interface");

    iAutoControlSettings->setAeLock(true);
    iAutoControlSettings->setIspDigitalGainRange(Range<float>(1,1));
    iSourceSettings->setSensorMode(m_sensorModeID);
    if (g_masterDev)
	iSourceSettings->setFrameDurationRange(1e9/DEFAULT_FPS);
    else
    	iSourceSettings->setFrameDurationRange(iSensorMode->getFrameDurationRange().min());

    iSourceSettings->setGainRange(iSensorMode->getAnalogGainRange());
    iSourceSettings->setExposureTimeRange(
		Range<uint64_t>(iSensorMode->getExposureTimeRange().min(),
				iSensorMode->getFrameDurationRange().max()));

    return true;
}


CameraModulesEGL::~CameraModulesEGL()
{
    if (m_outputStream) {
       if (m_request) {
           IRequest *iRequest = interface_cast<IRequest>(m_request);
           iRequest->disableOutputStream(m_outputStream.get());
           IEGLOutputStream *iStream = interface_cast<IEGLOutputStream>(m_outputStream.get());
           iStream->disconnect();

           //Destroy Output Stream
           m_outputStream.reset();
        }

    }
}

}; //namespace ArgusSamples

