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
#include "userAEAlgorithm.h"
#include <algorithm>
#include <math.h>
#include <cmath>

namespace ArgusSamples
{

bool userAEAlgorithm::threadInitialize()
{
    ICaptureSession *iSession = interface_cast<ICaptureSession>(m_cameraModule->getSession());
    if (!iSession)
	ORIGINATE_ERROR("Failed to get ICapture Interface");

    ISensorMode *iSensorMode = interface_cast<ISensorMode>(m_cameraModule->getSensorMode()); 
    if (!iSensorMode)
	ORIGINATE_ERROR("Failed to get ISensor Mode interface");

    std::vector<EventType> eventTypes;
    eventTypes.push_back(Argus::EVENT_TYPE_CAPTURE_COMPLETE);

    IEventProvider *iEventProvider = interface_cast<IEventProvider>(m_cameraModule->getSession());
    if (!iEventProvider)
	ORIGINATE_ERROR("Failed to get iEvent Provider interface");

    m_eventQueue.reset(iEventProvider->createEventQueue(eventTypes));
    
    // Update Exposure parameters 
    m_limitExposureTimeRange.min() = m_lightSource ? 10000000 : 8333333;
    m_limitExposureTimeRange.max() = iSensorMode->getFrameDurationRange().min();
    m_sensorResolution = iSensorMode->getResolution();
    m_sensorGainRange = iSensorMode->getAnalogGainRange();
    m_totalPixels = (uint64_t)(iSensorMode->getResolution().area() * 0.998);
    IRequest *iRequest = interface_cast<IRequest>(m_cameraModule->getRequest());
    if (!iRequest)
	ORIGINATE_ERROR("Failed to get Irequest interface");

    m_iSourceSettings = interface_cast<ISourceSettings>(iRequest->getSourceSettings());

    return true;
}

bool userAEAlgorithm::threadExecute()
{
    IEventProvider *iEventProvider = interface_cast<IEventProvider>(m_cameraModule->getSession());
    if (!iEventProvider)
	ORIGINATE_ERROR("Failed to get iEvent Provider interface");

    iEventProvider->waitForEvents(m_eventQueue.get());

    IEventQueue *iEventQueue = interface_cast<IEventQueue>(m_eventQueue.get());
    if (!iEventQueue)
	ORIGINATE_ERROR("Failed to get iEventQueue");

    for (uint32_t i = 0; i < iEventQueue->getSize(); i++)
    {
	const Event *event = iEventQueue->getEvent(i);
	const IEvent *iEvent = interface_cast<const IEvent>(event);
	if (!iEvent)
	    ORIGINATE_ERROR("Failed to get IEvent interface");

	if (!g_streamState) {
	    requestShutdown();
	    break;
	}

	if (iEvent->getEventType() == EVENT_TYPE_CAPTURE_COMPLETE)
	{
	    const IEventCaptureComplete *iEventComplete = interface_cast<const IEventCaptureComplete>(event);
	    const CaptureMetadata *metadata = iEventComplete->getMetadata();
	    const ICaptureMetadata* iMetadata = interface_cast<const ICaptureMetadata>(metadata);
	    if (!iMetadata)
		ORIGINATE_ERROR("Failed to get Sensor Metadata");

	    uint64_t frameExposureTime = iMetadata->getSensorExposureTime();
	    float curExposureLevel, curExposureInMS, Delta_Deviation, ShadowProbability = 0, HighlightProbability = 0;
	    curExposureInMS = iMetadata->getSensorExposureTime()/1e9;

	    const IRGBHistogram *rgbHistogram = interface_cast<const IRGBHistogram>(iMetadata->getRGBHistogram());
	    if (!rgbHistogram)
		    ORIGINATE_ERROR("Failed to get RGB Histogram interface");

	    std::vector<RGBTuple<uint32_t> > histogram;
	    if (rgbHistogram->getHistogram(&histogram) != STATUS_OK)
		    ORIGINATE_ERROR("Failed to get RGB Histogram");

	    for (uint64_t i = 0; i < histogram.size(); i++) {
		    float Prob_Distribution;
		    m_LuminanceHistogram[i] = (float)(histogram[i].r() +
				    histogram[i].g() +
				    histogram[i].b()) / 3;

		    Prob_Distribution = m_LuminanceHistogram[i] / m_totalPixels;

		    if ( i <= (histogram.size()/2))
			    ShadowProbability += Prob_Distribution;

		    else
			    HighlightProbability += Prob_Distribution;
	    }

	    Delta_Deviation = ShadowProbability - HighlightProbability;

	    float absoluteDeviation = abs(Delta_Deviation*100);

	    if (absoluteDeviation <= 10)
		    continue;

	    else {
		    float exposureChange = Delta_Deviation * curExposureInMS * 0.5f;
		    curExposureLevel = (float) curExposureInMS + exposureChange;

		    frameExposureTime = curExposureLevel * 1e9;

		    if (frameExposureTime >= m_limitExposureTimeRange.min()) 
			    frameExposureTime = std::min((uint64_t)(curExposureLevel * 1e9), m_limitExposureTimeRange.max());

		    else if (frameExposureTime >= m_limitExposureTimeRange.min() * FLICKERSENSITIVITY) 
			    frameExposureTime = std::max((uint64_t)(curExposureLevel * 1e9), m_limitExposureTimeRange.min());


		    if (frameExposureTime != m_prevExposure) {
			if (m_iSourceSettings->setExposureTimeRange(Range<uint64_t>(frameExposureTime)) != STATUS_OK)
			    ORIGINATE_ERROR("Failed to set Exposure Time");

			m_controlUpdated = true;
		    	m_prevExposure = frameExposureTime;
		    }
	    }
	    if (!g_calibrate && m_controlUpdated) {
		    ICaptureSession *iSession = interface_cast<ICaptureSession>(m_cameraModule->getSession());
		    if (!iSession)
			    ORIGINATE_ERROR("Cannot get Session interface");

		    iSession->repeat(m_cameraModule->getRequest());
		    m_controlUpdated = false;
	    }
	}
    }
    return true;
}

bool userAEAlgorithm::threadShutdown()
{
    m_eventQueue.reset();

    return true;
}

}; //namespace ArgusSamples
