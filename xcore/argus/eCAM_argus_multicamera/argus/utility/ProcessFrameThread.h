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

/*******************************************************************************
 * Argus Consumer Thread:
 * This is the thread acquires buffers from each stream and composite them to
 * one frame. Finally it renders the composited frame through EGLRenderer if
 * preview is enabled. Also this frame is processed based on customer input
 ******************************************************************************/
#ifndef PROCESSFRAME_H
#define PROCESSFRAME_H

#include "Error.h"
#include "Thread.h"
#include "VideoEncoder.h"
#include "CameraModuleEGL.h"

#include <Argus/Argus.h>
#include <nvbuf_utils.h>
#include <NvEglRenderer.h>
#include <EGLStream/EGLStream.h>
#include <EGLStream/NV/ImageNativeBuffer.h>

#define MAX_CAMERAS 6

using namespace Argus;
using namespace EGLStream;

namespace ArgusSamples
{

class ProcessFrameThread : public Thread
{
public:
    explicit ProcessFrameThread(std::vector<CameraModulesEGL*> &streams, 
		Rectangle<uint32_t> WindowSize, NvEglRenderer *renderer) :
        m_cameraModules(streams),
	m_windowSize(WindowSize),
	m_renderer(renderer),
        m_compositedFrame(0),
	total_frames(0),
	out_sync(0)
    {
    }
    virtual ~ProcessFrameThread();

protected:
    /* APIS to configure and save frames */
    bool SaveImagefile(Image *image);

    CaptureMetadata *GetStreamMetadata(Frame *frame);

    bool BurstCapture(std::vector<Image*> images);

    bool enableStreaming();

    bool disableStreaming();

    bool BurstRecordStart();

    bool BurstRecordStop();

    bool VideoRecordStart();

    bool VideoRecordStop();

    bool Check_Sync(std::vector<Frame*> frames, uint8_t *skip);

    bool DisplayFrameMetadata(CaptureMetadata *Metadata);

    /** @name Thread methods */
    /**@{*/
    virtual bool threadInitialize();
    virtual bool threadExecute();
    virtual bool threadShutdown();
    /**@}*/

    VideoRecord *videoRecorder[MAX_CAMERAS];
    std::vector<CameraModulesEGL*> &m_cameraModules;
    Rectangle<uint32_t> m_windowSize;
    UniqueObj<FrameConsumer> m_consumers[MAX_CAMERAS];
    int m_dmabufs[MAX_CAMERAS];
    NvBufferCompositeParams m_compositeParam;
    NvEglRenderer *m_renderer;
    int m_compositedFrame;
    uint32_t tmp_devID;
    uint64_t out_sync, total_frames;
};

}; //Namespace ArgusSamples

#endif //PROCESSFRAME_H
