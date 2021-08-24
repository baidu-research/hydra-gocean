/*
 * Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
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

#include "video_dec_drm.h"

#ifndef USE_NVBUF_TRANSFORM_API
using namespace std;

unordered_map <int, int> fd_map;

void *
renderer_dequeue_loop_fcn(void *args)
{
    context_t *ctx = (context_t *) args;
    int fd;
    int index;

    while (true)
    {
        fd = ctx->drm_renderer->dequeBuffer();

        // Received EOS signal from drm rendering thread
        if (fd == -1)
            return NULL;

        // Return the buffer back into converter capture plane in time
        // to avoid starving it
        auto map_entry = fd_map.find(fd);
        if (map_entry != fd_map.end())
        {
            index = (int) map_entry->second;
            fd_map.erase(fd);
            struct v4l2_buffer buf;
            struct v4l2_plane planes[MAX_PLANES];

            memset (&buf, 0 , sizeof(buf));
            memset(planes, 0, sizeof(planes));
            buf.index = index;
            buf.m.planes = planes;
            buf.m.planes[0].m.fd = fd;

            if (ctx->conv->capture_plane.qBuffer(buf, NULL) < 0)
            {
                cerr << "Error while enqueueing conv capture plane buffer" << endl;
                abort(ctx);
                return NULL;
            }
        }
        else
        {
            cerr << "Error while retrieving the fd from map list" << endl;
            abort(ctx);
            return NULL;
        }
    };

    return NULL;
}

static bool
conv_output_dqbuf_thread_callback(struct v4l2_buffer *v4l2_buf,
                                   NvBuffer * buffer, NvBuffer * shared_buffer,
                                   void *arg)
{
    context_t *ctx = (context_t *) arg;
    struct v4l2_buffer dec_capture_ret_buffer;
    struct v4l2_plane planes[MAX_PLANES];

    if (!v4l2_buf)
    {
        cerr << "Error while dequeueing conv output plane buffer" << endl;
        abort(ctx);
        return false;
    }

    if (v4l2_buf->m.planes[0].bytesused == 0)
    {
        return false;
    }

    memset(&dec_capture_ret_buffer, 0, sizeof(dec_capture_ret_buffer));
    memset(planes, 0, sizeof(planes));

    dec_capture_ret_buffer.index = shared_buffer->index;
    dec_capture_ret_buffer.m.planes = planes;

    pthread_mutex_lock(&ctx->queue_lock);
    ctx->conv_output_plane_buf_queue->push(buffer);

    // Return the buffer dequeued from converter output plane
    // back to decoder capture plane
    if (ctx->dec->capture_plane.qBuffer(dec_capture_ret_buffer, NULL) < 0)
    {
        abort(ctx);
        pthread_mutex_unlock(&ctx->queue_lock);
        return false;
    }

    pthread_cond_broadcast(&ctx->queue_cond);
    pthread_mutex_unlock(&ctx->queue_lock);

    return true;
}

static bool
conv_capture_dqbuf_thread_callback(struct v4l2_buffer *v4l2_buf,
                                    NvBuffer * buffer, NvBuffer * shared_buffer,
                                    void *arg)
{
    context_t *ctx = (context_t *) arg;

    if (!v4l2_buf)
    {
        cerr << "Error while dequeueing conv capture plane buffer" << endl;
        abort(ctx);
        return false;
    }

    if (v4l2_buf->m.planes[0].bytesused == 0)
    {
        // Enqueue an invalid fd to send EOS signal to drm rendering thread
        ctx->drm_renderer->enqueBuffer(-1);
        return false;
    }

    // Enqueue the fd into drm rendering queue
    // fd_map is used to store the context of v4l2 buff
    fd_map.insert(make_pair(buffer->planes[0].fd, v4l2_buf->index));
    ctx->drm_renderer->enqueBuffer(buffer->planes[0].fd);

    // To fix the flicker issue(Bug 200292247), we seperate the
    // the buffer enqueue process from this callback into
    // the thread renderer_dequeue_loop and make it running
    // asynchronously.

    return true;
}

int conv_send_eos(context_t *ctx)
{
    // Check if converter is running
    if (ctx->conv->output_plane.getStreamStatus())
    {
        NvBuffer *conv_buffer;
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(&planes, 0, sizeof(planes));

        v4l2_buf.m.planes = planes;
        pthread_mutex_lock(&ctx->queue_lock);
        while (ctx->conv_output_plane_buf_queue->empty())
        {
            pthread_cond_wait(&ctx->queue_cond, &ctx->queue_lock);
        }
        conv_buffer = ctx->conv_output_plane_buf_queue->front();
        ctx->conv_output_plane_buf_queue->pop();
        pthread_mutex_unlock(&ctx->queue_lock);

        v4l2_buf.index = conv_buffer->index;

        // Queue EOS buffer on converter output plane
        return ctx->conv->output_plane.qBuffer(v4l2_buf, NULL);
    }

    return 0;
}

int conv_initialize(context_t *ctx)
{
    ctx->conv_output_plane_buf_queue = new queue < NvBuffer * >;
    pthread_mutex_init(&ctx->queue_lock, NULL);
    pthread_cond_init(&ctx->queue_cond, NULL);

    ctx->conv = NvVideoConverter::createVideoConverter("conv0");
    if (!ctx->conv) {
        cerr << "Could not create video converter" << endl;
        return -1;
    }
    ctx->conv->output_plane.
        setDQThreadCallback(conv_output_dqbuf_thread_callback);
    ctx->conv->capture_plane.
        setDQThreadCallback(conv_capture_dqbuf_thread_callback);

    return 0;
}

void conv_destroy(context_t *ctx)
{
    delete ctx->conv;
    pthread_mutex_destroy(&ctx->queue_lock);
    pthread_cond_destroy(&ctx->queue_cond);
    delete ctx->conv_output_plane_buf_queue;
}

int conv_reset(context_t *ctx)
{
    int ret = 0;
    ret = conv_send_eos(ctx);
    if (ret < 0)
        return ret;

    ctx->conv->capture_plane.waitForDQThread(2000);

    ctx->conv->output_plane.deinitPlane();
    ctx->conv->capture_plane.deinitPlane();

    while(!ctx->conv_output_plane_buf_queue->empty())
    {
        ctx->conv_output_plane_buf_queue->pop();
    }

    return ret;
}

int conv_configure(context_t *ctx,
                   struct v4l2_format format,
                   struct v4l2_crop crop)
{
    int ret = 0;
    NvVideoDecoder *dec = ctx->dec;

    ret = ctx->conv->setOutputPlaneFormat(format.fmt.pix_mp.pixelformat,
                                          format.fmt.pix_mp.width,
                                          format.fmt.pix_mp.height,
                                          V4L2_NV_BUFFER_LAYOUT_BLOCKLINEAR);
    if (ret < 0) {
        cerr << "Error in converter output plane set format" << endl;
        return ret;
    }

    ret = ctx->conv->setCapturePlaneFormat(format.fmt.pix_mp.pixelformat,
                                            crop.c.width,
                                            crop.c.height,
                                            V4L2_NV_BUFFER_LAYOUT_PITCH);
    if (ret < 0) {
        cerr << "Error in converter capture plane set format" << endl;
        return ret;
    }

    ret = ctx->conv->setCropRect(0, 0, crop.c.width, crop.c.height);
    if (ret < 0) {
        cerr << "Error while setting crop rect" << endl;
        return ret;
    }

    ret =
        ctx->conv->output_plane.setupPlane(V4L2_MEMORY_DMABUF,
                                            dec->capture_plane.
                                            getNumBuffers(), false, false);
    if (ret < 0) {
        cerr << "Error in converter output plane setup" << endl;
        return ret;
    }

    ret =
        ctx->conv->capture_plane.setupPlane(V4L2_MEMORY_MMAP,
                                             dec->capture_plane.
                                             getNumBuffers(), true, false);
    if (ret < 0) {
        cerr << "Error in converter capture plane setup" << endl;
        return ret;
    }

    ret = ctx->conv->output_plane.setStreamStatus(true);
    if (ret < 0) {
        cerr << "Error in converter output plane streamon" << endl;
        return ret;
    }

    ret = ctx->conv->capture_plane.setStreamStatus(true);
    if (ret < 0) {
        cerr << "Error in converter capture plane streamon" << endl;
        return ret;
    }

    // Add all empty conv output plane buffers to conv_output_plane_buf_queue
    for (uint32_t i = 0; i < ctx->conv->output_plane.getNumBuffers(); i++)
    {
        ctx->conv_output_plane_buf_queue->push(ctx->conv->output_plane.
                getNthBuffer(i));
    }

    for (uint32_t i = 0; i < ctx->conv->capture_plane.getNumBuffers(); i++)
    {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, sizeof(planes));

        v4l2_buf.index = i;
        v4l2_buf.m.planes = planes;
        ret = ctx->conv->capture_plane.qBuffer(v4l2_buf, NULL);
        if (ret < 0) {
            cerr << "Error Qing buffer at converter capture plane" << endl;
            return ret;
        }
    }
    ctx->conv->output_plane.startDQThread(ctx);
    ctx->conv->capture_plane.startDQThread(ctx);

    return ret;
}

int conv_qBuffer(context_t *ctx, NvBuffer *dec_buffer)
{
    NvBuffer *conv_buffer;
    struct v4l2_buffer conv_output_buffer;
    struct v4l2_plane conv_planes[MAX_PLANES];

    // Give the buffer to video converter output plane
    // instead of returning the buffer back to decoder capture plane
    memset(&conv_output_buffer, 0, sizeof(conv_output_buffer));
    memset(conv_planes, 0, sizeof(conv_planes));
    conv_output_buffer.m.planes = conv_planes;

    // Get an empty conv output plane buffer from conv_output_plane_buf_queue
    pthread_mutex_lock(&ctx->queue_lock);
    while (ctx->conv_output_plane_buf_queue->empty())
    {
        pthread_cond_wait(&ctx->queue_cond, &ctx->queue_lock);
    }
    conv_buffer = ctx->conv_output_plane_buf_queue->front();
    ctx->conv_output_plane_buf_queue->pop();
    pthread_mutex_unlock(&ctx->queue_lock);

    conv_output_buffer.index = conv_buffer->index;

    return ctx->conv->output_plane.
            qBuffer(conv_output_buffer, dec_buffer);
}
#endif
