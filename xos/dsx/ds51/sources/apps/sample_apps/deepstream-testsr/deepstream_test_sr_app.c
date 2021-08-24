/*
 * Copyright (c) 2020-2021, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <gst/gst.h>
#include <glib.h>
#include <stdio.h>
#include <string.h>
#include "gst-nvdssr.h"

GST_DEBUG_CATEGORY (NVDS_APP);

#define MAX_DISPLAY_LEN 64

#define PGIE_CLASS_ID_VEHICLE 0
#define PGIE_CLASS_ID_PERSON 2

/* The muxer output resolution must be set if the input streams will be of
 * different resolution. The muxer will scale all the input frames to this
 * resolution. */
#define MUXER_OUTPUT_WIDTH 1920
#define MUXER_OUTPUT_HEIGHT 1080

/* Muxer batch formation timeout, for e.g. 40 millisec. Should ideally be set
 * based on the fastest source's framerate. */
#define MUXER_BATCH_TIMEOUT_USEC 40000

/* By default, OSD process-mode is set to CPU_MODE. To change mode, set as:
 * 1: GPU mode (for Tesla only)
 * 2: HW mode (For Jetson only)
 */
#define OSD_PROCESS_MODE 0

/* By default, OSD will not display text. To display text, change this to 1 */
#define OSD_DISPLAY_TEXT 0

gint frame_number = 0;
gchar pgie_classes_str[4][32] = { "Vehicle", "TwoWheeler", "Person",
  "Roadsign"
};

/* Config file parameters used for recording
 * User needs to change these parameters to reflect the change in recordings
 * e.g duration, start-time etc. */

/* Container format of recorded file 0 for mp4 and 1 for mkv format
 */
#define SMART_REC_CONTAINER 0

/* Cache functionality of recording
 */
#define VIDEO_CACHE_SIZE 15

/* Timeout if duration of recording is not set by user
 */
#define SMART_REC_DEFAULT_DURATION 10

/* Time at which it recording is started
 */
#define START_TIME 2

/* Duration of recording
 */
#define SMART_REC_DURATION 7

static gboolean bbox_enabled = TRUE;

GOptionEntry entries[] = {
  {"enable", 'e', 0, G_OPTION_ARG_INT, &bbox_enabled,
      "Display Bounding box in recorded files", NULL}
  ,
  {NULL}
  ,
};

static gboolean
bus_call (GstBus * bus, GstMessage * msg, gpointer data)
{
  GMainLoop *loop = (GMainLoop *) data;
  switch (GST_MESSAGE_TYPE (msg)) {
    case GST_MESSAGE_EOS:
      g_print ("End of stream\n");
      g_main_loop_quit (loop);
      break;
    case GST_MESSAGE_ERROR:{
      gchar *debug;
      GError *error;
      gst_message_parse_error (msg, &error, &debug);
      g_printerr ("ERROR from element %s: %s\n",
          GST_OBJECT_NAME (msg->src), error->message);
      if (debug)
        g_printerr ("Error details: %s\n", debug);
      g_free (debug);
      g_error_free (error);
      g_main_loop_quit (loop);
      break;
    }
    default:
      break;
  }
  return TRUE;
}

static gpointer
smart_record_callback (NvDsSRRecordingInfo * info, gpointer userData)
{
  static GMutex mutex;
  FILE *logfile = NULL;
  g_return_val_if_fail (info, NULL);

  g_mutex_lock (&mutex);
  logfile = fopen ("smart_record.log", "a");
  if (logfile) {
    fprintf (logfile, "%d:%d:%d:%ldms:%s:%s\n",
        info->sessionId, info->width, info->height, info->duration,
        info->dirpath, info->filename);
    fclose (logfile);
  } else {
    g_print ("Error in opeing smart record log file\n");
  }
  g_mutex_unlock (&mutex);

  return NULL;
}

static gboolean
smart_record_event_generator (gpointer data)
{
  NvDsSRSessionId sessId = 0;
  NvDsSRContext *ctx = (NvDsSRContext *) data;
  guint startTime = START_TIME;
  guint duration = SMART_REC_DURATION;

  if (ctx->recordOn) {
    g_print ("Recording done.\n");
    if (NvDsSRStop (ctx, 0) != NVDSSR_STATUS_OK)
      g_printerr ("Unable to stop recording\n");
  } else {
    g_print ("Recording started..\n");
    if (NvDsSRStart (ctx, &sessId, startTime, duration,
            NULL) != NVDSSR_STATUS_OK)
      g_printerr ("Unable to start recording\n");
  }
  return TRUE;
}

static void
cb_newpad (GstElement * element, GstPad * element_src_pad, gpointer data)
{

  g_print ("In cb_newpad\n");
  GstCaps *caps = gst_pad_get_current_caps (element_src_pad);
  const GstStructure *str = gst_caps_get_structure (caps, 0);
  const gchar *name = gst_structure_get_name (str);

  GstElement *depay_elem = (GstElement *) data;

  const gchar *media = gst_structure_get_string (str, "media");
  gboolean is_video = (!g_strcmp0 (media, "video"));

  /* Need to check if the pad created by the source is for video and not
   * audio. */
  if (g_strrstr (name, "x-rtp") && is_video) {

    GstPad *sinkpad = gst_element_get_static_pad (depay_elem, "sink");
    if (gst_pad_link (element_src_pad, sinkpad) != GST_PAD_LINK_OK) {
      g_printerr ("Failed to link depay loader to rtsp src");
    }
    gst_object_unref (sinkpad);
  }
  gst_caps_unref (caps);
}

int
main (int argc, char *argv[])
{
  GMainLoop *loop = NULL;
  GstElement *pipeline = NULL, *streammux = NULL, *sink = NULL, *pgie = NULL,
      *source = NULL, *nvvidconv = NULL, *nvvidconv2 = NULL,
      *encoder_post_osd = NULL, *queue_pre_sink = NULL,
      *queue_post_osd = NULL, *parser_post_osd = NULL, *parser_pre_recordbin =
      NULL, *nvosd = NULL, *tee_post_osd = NULL, *tee_pre_decode =
      NULL, *parser_pre_decode = NULL, *depay_pre_decode = NULL, *decoder =
      NULL;

  GstCaps *caps = NULL;
  GstElement *cap_filter = NULL;

  GstBus *bus = NULL;
  guint bus_watch_id = 0;
  guint i = 0, num_sources = 1;

  guint pgie_batch_size = 0;

#ifdef PLATFORM_TEGRA
  GstElement *transform = NULL;
#endif
  GOptionContext *gctx = NULL;
  GOptionGroup *group = NULL;
  GError *error = NULL;

  NvDsSRContext *ctx = NULL;
  NvDsSRInitParams params = { 0 };

  gctx = g_option_context_new ("Nvidia DeepStream Test-SR app");
  group = g_option_group_new ("SR_test", NULL, NULL, NULL, NULL);
  g_option_group_add_entries (group, entries);

  g_option_context_set_main_group (gctx, group);
  g_option_context_add_group (gctx, gst_init_get_option_group ());

  GST_DEBUG_CATEGORY_INIT (NVDS_APP, "NVDS_APP", 0, NULL);

  if (!g_option_context_parse (gctx, &argc, &argv, &error)) {
    g_printerr ("%s", error->message);
    g_print ("%s", g_option_context_get_help (gctx, TRUE, NULL));
    return -1;
  }

  /* Check input arguments */
  if (argc < 2) {
    g_printerr ("Usage: %s <rtsp_h264 uri>\n", argv[0]);
    return -1;
  }

  if (argc > 2) {
    g_printerr ("One rtsp_h264 uri supported Usage: %s <rtsp_h264 uri> \n",
        argv[0]);
    return -1;
  }

  /* Standard GStreamer initialization */
  gst_init (&argc, &argv);
  loop = g_main_loop_new (NULL, FALSE);

  /* Create gstreamer elements */
  /* Create Pipeline element that will form a connection of other elements */
  pipeline = gst_pipeline_new ("dstest-sr-pipeline");

  source = gst_element_factory_make ("rtspsrc", "rtsp-source");
  g_object_set (G_OBJECT (source), "location", argv[1], NULL);


  depay_pre_decode = gst_element_factory_make ("rtph264depay", "h264-depay");

  parser_pre_decode = gst_element_factory_make ("h264parse", "h264-parse");

  if (!source || !depay_pre_decode || !parser_pre_decode) {
    g_printerr ("One element in source end could not be created.\n");
    return -1;
  }

  g_signal_connect (G_OBJECT (source), "pad-added",
      G_CALLBACK (cb_newpad), depay_pre_decode);
  /* Create tee which connects decoded source data and Smart record bin without bbox */
  tee_pre_decode = gst_element_factory_make ("tee", "tee-pre-decode");

  decoder = gst_element_factory_make ("nvv4l2decoder", "nvv4l2-decoder");

  streammux = gst_element_factory_make ("nvstreammux", "stream-muxer");

  /* Use nvinfer to infer on batched frame. */
  pgie = gst_element_factory_make ("nvinfer", "primary-nvinference-engine");

  /* Use queue to connect to the sink after tee_post_osd element */
  queue_pre_sink = gst_element_factory_make ("queue", "queue-pre-sink");

  /* Use convertor to convert from NV12 to RGBA as required by nvosd */
  nvvidconv = gst_element_factory_make ("nvvideoconvert", "nvvideo-converter");

  /* Use convertor to convert from RGBA to CAPS filter data format */
  nvvidconv2 =
      gst_element_factory_make ("nvvideoconvert", "nvvideo-converter2");

  /* Create OSD to draw on the converted RGBA buffer */
  nvosd = gst_element_factory_make ("nvdsosd", "nv-onscreendisplay");

  /* Create tee which connects to sink and Smart record bin with bbox */
  tee_post_osd = gst_element_factory_make ("tee", "tee-post-osd");


  /* Finally render the osd output */
#ifdef PLATFORM_TEGRA

  transform = gst_element_factory_make ("nvegltransform", "nvegl-transform");
  if (!transform) {
    g_printerr ("One tegra element could not be created. Exiting.\n");
    return -1;
  }
#endif

  sink = gst_element_factory_make ("nveglglessink", "nvvideo-renderer");
  g_object_set (G_OBJECT (sink), "async", FALSE, NULL);
  g_object_set (G_OBJECT (streammux), "live-source", 1, NULL);

  caps = gst_caps_from_string ("video/x-raw(memory:NVMM), format=(string)I420");
  cap_filter =
      gst_element_factory_make ("capsfilter", "src_cap_filter_nvvidconv");
  g_object_set (G_OBJECT (cap_filter), "caps", caps, NULL);
  gst_caps_unref (caps);

  if (!pgie || !nvvidconv || !nvosd || !nvvidconv2 || !cap_filter
      || !tee_post_osd || !tee_pre_decode || !sink) {
    g_printerr ("One element could not be created. Exiting.\n");
    return -1;
  }

  g_object_set (G_OBJECT (streammux), "batch-size", num_sources, NULL);

  g_object_set (G_OBJECT (streammux), "width", MUXER_OUTPUT_WIDTH, "height",
      MUXER_OUTPUT_HEIGHT,
      "batched-push-timeout", MUXER_BATCH_TIMEOUT_USEC, NULL);

  /* Configure the nvinfer element using the nvinfer config file. */
  g_object_set (G_OBJECT (pgie),
      "config-file-path", "dstestsr_pgie_config.txt", NULL);

  /* Override the batch-size set in the config file with the number of sources. */
  g_object_get (G_OBJECT (pgie), "batch-size", &pgie_batch_size, NULL);
  if (pgie_batch_size != num_sources) {
    g_printerr
        ("WARNING: Overriding infer-config batch-size (%d) with number of sources (%d)\n",
        pgie_batch_size, num_sources);
    g_object_set (G_OBJECT (pgie), "batch-size", num_sources, NULL);
  }

  g_object_set (G_OBJECT (nvosd), "process-mode", OSD_PROCESS_MODE,
      "display-text", OSD_DISPLAY_TEXT, NULL);

  g_object_set (G_OBJECT (sink), "qos", 0, NULL);

  /* we add a message handler */
  bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
  bus_watch_id = gst_bus_add_watch (bus, bus_call, loop);
  gst_object_unref (bus);

  /* Set up the pipeline
   * we add all elements into the pipeline
   * rtsp-source-> h264-depay -> tee-> h264-parser -> decoder ->nvstreammux -> nvinfer -> nvvidconv -> nvosd -> nvvidconv -> caps_filter -> tee -> queue -> video-renderer
   *                                |-> h264-parser -> recordbin                                                                               |-> queue -> encoder -> parser -> recordbin
   */
  gst_bin_add_many (GST_BIN (pipeline), source, depay_pre_decode,
      tee_pre_decode, parser_pre_decode, decoder, streammux, pgie, nvvidconv,
      nvosd, nvvidconv2, cap_filter, tee_post_osd, queue_pre_sink, sink, NULL);

#ifdef PLATFORM_TEGRA
  gst_bin_add (GST_BIN (pipeline), transform);
#endif

  /* Link the elements together till decoder */
  if (!gst_element_link_many (depay_pre_decode, tee_pre_decode,
          parser_pre_decode, decoder, NULL)) {
    g_printerr ("Elements could not be linked: 1. Exiting.\n");
    return -1;
  }

  /* Link decoder with streammux */
  GstPad *sinkpad, *srcpad;
  gchar pad_name_sink[16] = "sink_0";
  gchar pad_name_src[16] = "src";

  sinkpad = gst_element_get_request_pad (streammux, pad_name_sink);
  if (!sinkpad) {
    g_printerr ("Streammux request sink pad failed. Exiting.\n");
    return -1;
  }

  srcpad = gst_element_get_static_pad (decoder, pad_name_src);
  if (!srcpad) {
    g_printerr ("Decoder request src pad failed. Exiting.\n");
    return -1;
  }

  if (gst_pad_link (srcpad, sinkpad) != GST_PAD_LINK_OK) {
    g_printerr ("Failed to link decoder to stream muxer. Exiting.\n");
    return -1;
  }

  gst_object_unref (sinkpad);
  gst_object_unref (srcpad);

  /* Link the remaining elements of the pipeline to streammux */
#ifdef PLATFORM_TEGRA
  if (!gst_element_link_many (streammux, pgie,
          nvvidconv, nvosd, nvvidconv2, cap_filter, tee_post_osd,
          queue_pre_sink, transform, sink, NULL)) {
    g_printerr ("Elements could not be linked. Exiting.\n");
    return -1;
  }
#else
  if (!gst_element_link_many (streammux, pgie,
          nvvidconv, nvosd, nvvidconv2, cap_filter, tee_post_osd,
          queue_pre_sink, sink, NULL)) {
    g_printerr ("Elements could not be linked. Exiting.\n");
    return -1;
  }
#endif

  /* Parameters are set before creating record bin
   * User can set additional parameters e.g recorded file path etc.
   * Refer NvDsSRInitParams structure for additional parameters
   */
  params.containerType = SMART_REC_CONTAINER;
  params.videoCacheSize = VIDEO_CACHE_SIZE;
  params.defaultDuration = SMART_REC_DEFAULT_DURATION;
  params.callback = smart_record_callback;
  params.fileNamePrefix = bbox_enabled ? "With_BBox" : "Without_BBox";

  if (NvDsSRCreate (&ctx, &params) != NVDSSR_STATUS_OK) {
    g_printerr ("Failed to create smart record bin");
    return -1;
  }

  if (!bbox_enabled) {

    /* Parse the encoded data after decoding raw data */
    parser_pre_recordbin =
        gst_element_factory_make ("h264parse", "parser-pre-recordbin");

    gst_bin_add_many (GST_BIN (pipeline), parser_pre_recordbin, ctx->recordbin,
        NULL);

    if (!gst_element_link_many (tee_pre_decode, parser_pre_recordbin,
            ctx->recordbin, NULL)) {
      g_print ("Elements not linked. Exiting. \n");
      return -1;
    }
  } else if (bbox_enabled) {
    /* Encode the data from tee before recording with bbox */
    encoder_post_osd =
        gst_element_factory_make ("nvv4l2h264enc", "encoder-post-osd");

    /* Parse the encoded data after osd component */
    parser_post_osd = gst_element_factory_make ("h264parse", "parser-post-osd");

    /* Use queue to connect the tee_post_osd to nvencoder */
    queue_post_osd = gst_element_factory_make ("queue", "queue-post-osd");

    gst_bin_add_many (GST_BIN (pipeline), queue_post_osd, encoder_post_osd,
        parser_post_osd, ctx->recordbin, NULL);

    if (!gst_element_link_many (tee_post_osd, queue_post_osd, encoder_post_osd,
            parser_post_osd, ctx->recordbin, NULL)) {
      g_print ("Elements not linked. Exiting. \n");
      return -1;
    }
  }

  if (ctx) {
    g_timeout_add (SMART_REC_DURATION * 1000, smart_record_event_generator,
        ctx);
  }

  /* Set the pipeline to "playing" state */
  g_print ("Now playing:");
  g_print (" %s", argv[i + 1]);

  g_print ("\n");
  gst_element_set_state (pipeline, GST_STATE_PLAYING);

  /* Wait till pipeline encounters an error or EOS */
  g_print ("Running...\n");
  g_main_loop_run (loop);
  if (pipeline && ctx) {
    if(NvDsSRDestroy (ctx) != NVDSSR_STATUS_OK)
    g_printerr ("Unable to destroy recording instance\n");
  }
  /* Out of the main loop, clean up nicely */
  g_print ("Returned, stopping playback\n");
  gst_element_set_state (pipeline, GST_STATE_NULL);
  g_print ("Deleting pipeline\n");
  gst_object_unref (GST_OBJECT (pipeline));
  g_source_remove (bus_watch_id);
  g_main_loop_unref (loop);
  return 0;
}
