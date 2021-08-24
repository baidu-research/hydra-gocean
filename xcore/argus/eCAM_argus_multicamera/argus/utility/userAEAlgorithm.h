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

#ifndef AEALGOL_H
#define AEALGOL_H

#include <Argus/Argus.h>
#include "CameraModuleEGL.h"

using namespace Argus;

namespace ArgusSamples
{

const float FLICKERSENSITIVITY = 0.5f;

class userAEAlgorithm : public Thread
{
public:
    explicit userAEAlgorithm(CameraModulesEGL *camModule, uint32_t lightSrc)
	: m_cameraModule(camModule)
	, m_lightSource(lightSrc)
	, m_sensorGainRange(0,30)
	, m_limitExposureTimeRange(0,0)
	, m_sensorResolution(1920, 1080)
	, m_prevExposure(0)
	, m_controlUpdated(false)
    {
    }
    virtual ~userAEAlgorithm()
    {
	if(m_eventQueue)
	    m_eventQueue.reset();
    }

protected:
    virtual bool threadInitialize();
    virtual bool threadExecute();
    virtual bool threadShutdown();

private:
    CameraModulesEGL *m_cameraModule;
    UniqueObj<OutputStream> m_outputStream;
    UniqueObj<EventQueue> m_eventQueue;
    ISourceSettings *m_iSourceSettings;
    Range<uint64_t> m_limitExposureTimeRange;
    Range<float> m_sensorGainRange;
    Size2D<uint32_t> m_sensorResolution;

protected:
    uint32_t m_FlickerFrequencyMode, m_lightSource;
    float m_LuminanceHistogram[256];
    uint64_t m_prevExposure, m_totalPixels;
    bool m_controlUpdated; 
};

}; // namespace ArgusSamples

#endif
