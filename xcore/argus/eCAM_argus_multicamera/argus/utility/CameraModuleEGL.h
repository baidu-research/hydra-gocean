/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION. All rights reserved.
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
#ifndef CAMERAMODULE_H
#define CAMERAMODULE_H

#include <Argus/Argus.h>

using namespace Argus;

namespace ArgusSamples
{

// An utility class to hold all resources of one capture session
class CameraModulesEGL : public Destructable
{
public:
    explicit CameraModulesEGL(CaptureSession *session);
    virtual ~CameraModulesEGL();

    bool initialize(CameraDevice *device, Argus::SensorMode* sensormode);

    bool setStreamProperties();

    CaptureSession* getSession() const
    {
        return m_captureSession;
    }

    OutputStream* getStream() const
    {
        return m_outputStream.get();
    }

    Request* getRequest() const
    {
        return m_request.get();
    }

    Argus::SensorMode *getSensorMode() const
    {
	return m_sensorModeID;
    }

    Argus::CameraDevice *getDevice() const
    {
	return m_camerDevice;
    }

    virtual void destroy()
    {
        delete this;
    }

private:
    CaptureSession 		*m_captureSession;
    UniqueObj<OutputStream> 	m_outputStream;
    UniqueObj<Request> 		m_request;
    Argus::SensorMode 		*m_sensorModeID;
    Argus::CameraDevice		*m_camerDevice;
    bool			m_initialized;
};

}; //namespace Argus Samples

#endif //CAMERAMODULE_H
