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
#ifndef CONTROLS_H
#define CONTROLS_H

#include <Argus/Argus.h>
#include "Value.h"
#include "Validator.h"
#include "CameraModuleEGL.h"

using namespace Argus;

namespace ArgusSamples
{
// valid denoise modes
static const ValidatorEnum<Argus::DenoiseMode>::ValueStringPair s_denoiseModes[] =
{
    { Argus::DENOISE_MODE_OFF, "off" },
    { Argus::DENOISE_MODE_FAST, "fast" },
    { Argus::DENOISE_MODE_HIGH_QUALITY, "high" }
};

// valid edge enhance modes
static const ValidatorEnum<Argus::EdgeEnhanceMode>::ValueStringPair s_edgeEnhanceModes[] =
{
    { Argus::EDGE_ENHANCE_MODE_OFF, "off" },
    { Argus::EDGE_ENHANCE_MODE_FAST, "fast" },
    { Argus::EDGE_ENHANCE_MODE_HIGH_QUALITY, "high" }
};

// valid AE antibanding modes
static const ValidatorEnum<Argus::AeAntibandingMode>::ValueStringPair s_aeAntibandingModes[] =
{
    { Argus::AE_ANTIBANDING_MODE_OFF, "off" },
    { Argus::AE_ANTIBANDING_MODE_AUTO, "auto" },
    { Argus::AE_ANTIBANDING_MODE_50HZ, "50hz" },
    { Argus::AE_ANTIBANDING_MODE_60HZ, "60hz" }
};

// valid AWB modes
static const ValidatorEnum<Argus::AwbMode>::ValueStringPair s_awbModes[] =
{
    { Argus::AWB_MODE_AUTO, "auto" },
    { Argus::AWB_MODE_INCANDESCENT, "incan" },
    { Argus::AWB_MODE_FLUORESCENT, "fluo" },
    { Argus::AWB_MODE_WARM_FLUORESCENT, "warm" },
    { Argus::AWB_MODE_DAYLIGHT, "day" },
    { Argus::AWB_MODE_CLOUDY_DAYLIGHT, "cloudy" },
    { Argus::AWB_MODE_TWILIGHT, "twil" },
    { Argus::AWB_MODE_SHADE, "shade" },
};

class ControlSettings : public Destructable
{
public:
    explicit ControlSettings(CameraModulesEGL *);

    ~ControlSettings()
    {
    }

    void ListControls();

    bool GetSetControls();

    bool ControlsInitialize();

    bool userSetSlaveFrameRate();

    bool SetDefaults();

    bool Start();

    virtual void destroy()
    {
	delete this;
    }

    std::string userGetAeAntibandingMode()
    {
	return m_aeAntibandingMode.toString();
    }

    bool userAeAntibandingMode();

    std::string userGetAeLock()
    {
	return m_aeLock.toString();
    }

    bool userAeLock();

    std::string userGetAwbMode()
    {
	return m_awbMode.toString();
    }

    bool userAwbMode();

    std::string userGetDenoiseMode()
    {
	return m_denoiseMode.toString();
    }

    bool userDenoiseMode();

    std::string userGetDenoiseStrength()
    {
	return m_denoiseStrength.toString();
    }

    bool userDenoiseStrength();

    std::string userGetEdgeEnhanceMode()
    {
	return m_edgeEnhanceMode.toString();
    }

    bool userEdgeEnhanceMode();

    std::string userGetEdgeEnhanceStrength()
    {
	return m_edgeEnhanceStrength.toString();
    }

    bool userEdgeEnhanceStrength();

    std::string userGetIspDigitalGainRange()
    {
	return m_ispDigitalGainRange.toString();
    }

    bool userIspDigitalGainRange();

    std::string userGetExposureTimeRange() 
    {
	return m_exposureTimeRange.toString();
    }

    bool userExposureCompensation();

    std::string userGetExposureCompensation()
    {
	return m_exposureCompensation.toString();
    }

    bool userExposureTimeRange();

    std::string userGetSensorAnalogGainRange()
    {
	return m_gainRange.toString();
    }

    bool userSensorAnalogGainRage();

    float userGetFrameRate()
    {
	return m_frameRate.get();
    }

    bool userFrameRate();

private:

    // Control Interfaces
    CameraModulesEGL		*m_cameraModule;
    IAutoControlSettings	*m_iAutoControlSettings;
    IEdgeEnhanceSettings	*m_iEdgeEnhanceSettings;
    Argus::SensorMode		*m_sensorModeIndex;
    IDenoiseSettings		*m_iDenoiseSettings;
    ISourceSettings		*m_iSourceControlSettings;

    Value<Argus::Range<Argus::Range<float> > >
        m_deviceIspDigitalGainRange;
    Value<Argus::Range<Argus::Range<uint64_t> > >
        m_sensorExposureTimeRange;
    Value<Argus::Range<Argus::Range<float> > >
        m_sensorAnalogGainRange;
    Value<Argus::Range<float> > m_sensorFrameRateRange;
    Value<Argus::Range<float> > m_deviceExposureCompensationRange;

    // ISP Controls
    Value<Argus::AeAntibandingMode>	m_aeAntibandingMode;
    Value<bool>				m_aeLock;
    Value<Argus::AwbMode>		m_awbMode;
    Value<Argus::DenoiseMode>		m_denoiseMode;
    Value<float>			m_denoiseStrength;
    Value<Argus::EdgeEnhanceMode>	m_edgeEnhanceMode;
    Value<float>			m_edgeEnhanceStrength;
    Value<Argus::Range<float> >		m_ispDigitalGainRange;
    Value<float> 			m_exposureCompensation;

    // Camera Controls
    Value<Argus::Range<uint64_t> >		m_exposureTimeRange;
    Value<Argus::Range<float> >			m_gainRange;
    Value<float>				m_frameRate;

    enum {
	CTRL_AEANTIBANDING_MODE = 1,
	CTRL_AE_LOCK,
	CTRL_AWB_MODE,
	CTRL_DENOISE_MODE,
	CTRL_DENOISE_STRENGTH,
	CTRL_EDGE_ENHANCE_MODE,
	CTRL_EDGE_ENHANCE_STRENGTH,
	CTRL_ISP_GAIN_RANGE,
	CTRL_EXPOSURE_COMPENSATION,
	CTRL_EXPOSURE_TIME_RANGE,
	CTRL_SENSOR_GAIN_RANGE,
	CTRL_FRAME_RATE,
    };
};

}; // NameSpace ArgusSamples

#endif
