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
#ifndef VIDEORECORD_H
#define VIDEORECORD_H

#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <EGLStream/NV/ImageNativeBuffer.h>

#include <gst/app/gstappsrc.h>
#include <NvVideoEncoder.h>
#include "CameraModuleEGL.h"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

using namespace Argus;
using namespace EGLStream;

// Constant configuration.
static const int    MAX_ENCODER_FRAMES = 5;

// Debug print macros.
#define PRODUCER_PRINT(...) printf("PRODUCER: " __VA_ARGS__)
#define CONSUMER_PRINT(...) printf("CONSUMER: " __VA_ARGS__)
#define CHECK_ERROR(expr) \
    do { \
        if ((expr) < 0) { \
            abort(); \
            ORIGINATE_ERROR(#expr " failed"); \
        } \
    } while (0);

namespace ArgusSamples
{

/*******************************************************************************
 * VideoEncoder thread:
 *   Creates an EGLStream::FrameConsumer object to read frames from the stream
 *   and create NvBuffers (dmabufs) from acquired frames before providing the
 *   buffers to V4L2 for video encoding. The encoder will save the encoded
 *   stream to disk.
 ******************************************************************************/
class VideoEncoderThread : public Thread
{
public:
    explicit VideoEncoderThread(OutputStream* stream, GstElement *appsrc_, SensorMode *sensorMode);
    ~VideoEncoderThread();

    bool isInError()
    {
        return m_gotError;
    }

private:
    /** @name Thread methods */
    /**@{*/
    virtual bool threadInitialize();
    virtual bool threadExecute();
    virtual bool threadShutdown();
    /**@}*/

    bool createVideoEncoder();
    void abort();

    static bool encoderCapturePlaneDqCallback(
            struct v4l2_buffer *v4l2_buf,
            NvBuffer *buffer,
            NvBuffer *shared_buffer,
            void *arg);

    OutputStream* m_stream;
    SensorMode*	  m_sensorMode;
    UniqueObj<FrameConsumer> m_consumer;
    CaptureMetadata *m_metadata;
    NvVideoEncoder *m_VideoEncoder;
    GstElement *m_appsrc_;
    GstClockTime timestamp;
    uint32_t timestamp_diff;
    bool m_gotError;
};

class RawVideoRecorder : public Thread
{
public:
    explicit RawVideoRecorder(OutputStream *stream, SensorMode *sensorMode);
    ~RawVideoRecorder();

private:
    /** @name Thread methods */
    /**@{*/
    virtual bool threadInitialize();
    virtual bool threadExecute();
    virtual bool threadShutdown();
    /**@}*/

    OutputStream *m_stream;
    SensorMode   *m_sensorMode;
    UniqueObj<FrameConsumer> m_consumer;
    FILE *fileptr;
    int m_fd;
    NvBufferParams m_params;
    std::ostringstream m_fileName;
    bool m_initialized, m_initOnce;
    size_t Buff_size[MAX_PLANES];
};

class VideoRecord 
{
public:
    VideoRecord(CameraModulesEGL *module)
	: m_cameraModule(module)
	, m_frameConsumer(NULL)
	, gst_pipeline(NULL)
	, err(NULL)
    {
    }
    ~VideoRecord()
    {
	if (m_outputStream)
	    m_outputStream.reset();
    }

    bool CreateVideoRecordStream();

    bool CloseVideoRecordStream();

protected:
    CameraModulesEGL *m_cameraModule;
    UniqueObj<OutputStream> m_outputStream;
    VideoEncoderThread *m_frameConsumer;
    RawVideoRecorder   *m_rawFrameConsumer;

    GMainLoop *main_loop;
    GstPipeline *gst_pipeline;
    GError *err;
    GstElement *appsrc_;
};

}; // namespace ArgusSamples

#endif //VIDEORECORD_H
