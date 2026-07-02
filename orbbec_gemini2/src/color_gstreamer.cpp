#include <libobsensor/ObSensor.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <iostream>

int main(int argc, char **argv)
try
{
    ob::Pipeline pipe;
    auto config = std::make_shared<ob::Config>();

    const int width = 1280, height = 720, fps = 30;
    config->enableVideoStream(OB_STREAM_COLOR, width, height, fps, OB_FORMAT_MJPG);
    pipe.start(config);

    gst_init(&argc, &argv);

    GstElement *pipeline = gst_pipeline_new("orbbec-gst");
    GstElement *appsrc = gst_element_factory_make("appsrc", "src");
    GstElement *jpegdec = gst_element_factory_make("jpegdec", "dec");
    GstElement *videoconvert = gst_element_factory_make("videoconvert", "conv");
    GstElement *caps_x264 = gst_element_factory_make("capsfilter", "caps_x264");
    GstElement *x264enc = gst_element_factory_make("x264enc", "enc");
    GstElement *h264parse = gst_element_factory_make("h264parse", "parse");
    GstElement *queue = gst_element_factory_make("queue", "queue");
    GstElement *sink = gst_element_factory_make("srtsink", "sink");

    if (!pipeline || !appsrc || !jpegdec || !videoconvert || !caps_x264 || !x264enc || !h264parse || !queue || !sink)
        return EXIT_FAILURE;

    GstCaps *caps = gst_caps_new_simple("image/jpeg",
                                        "width", G_TYPE_INT, width,
                                        "height", G_TYPE_INT, height,
                                        "framerate", GST_TYPE_FRACTION, fps, 1, NULL);

    g_object_set(appsrc,
                 "caps", caps,
                 "format", GST_FORMAT_TIME,
                 "is-live", TRUE,
                 "do-timestamp", FALSE,
                 "block", FALSE,
                 NULL);
    /* Ensure appsrc behaves as a live stream (not seekable) */
    g_object_set(appsrc, "stream-type", GST_APP_STREAM_TYPE_STREAM, NULL);
    gst_caps_unref(caps);

    /* Force x264 to receive canonical I420 raw frames */
    GstCaps *x264_caps = gst_caps_new_simple("video/x-raw",
                                             "format", G_TYPE_STRING, "I420",
                                             "framerate", GST_TYPE_FRACTION, fps, 1, NULL);
    g_object_set(caps_x264, "caps", x264_caps, NULL);
    gst_caps_unref(x264_caps);

    /* x264 encoder settings */
    g_object_set(x264enc,
                 "bitrate", 500,
                 NULL);

    const char *srt_uri = (argc > 1) ? argv[1] : "srt://192.168.1.XXX:7092?mode=caller";
    g_object_set(sink, "uri", srt_uri, "sync", FALSE, NULL);

    /* Configure queue to keep latency down and drop old buffers when overloaded */
    g_object_set(queue, "max-size-buffers", 2, "max-size-bytes", 0, "max-size-time", 0, "leaky", 2, NULL);

    gst_bin_add_many(GST_BIN(pipeline),
                     appsrc, jpegdec, videoconvert, caps_x264, x264enc, h264parse, queue, sink, NULL);

    if (!gst_element_link_many(appsrc, jpegdec, videoconvert, caps_x264, x264enc, h264parse, queue, sink, NULL))
    {
        std::cerr << "GStreamer: failed to link full pipeline (appsrc->jpegdec->videoconvert->caps->x264enc->h264parse->srtsink).\n";
        return EXIT_FAILURE;
    }

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    std::cout << "Press Ctrl+C to close the window and exit." << std::endl;

    guint64 frameCount = 0;

    while (true)
    {
        auto fs = pipe.waitForFrameset();
        if (!fs)
            continue;

        auto vf = fs->getFrame(OB_FRAME_COLOR)->as<const ob::VideoFrame>();
        if (!vf)
            continue;

        gsize size = vf->getDataSize();
        if (!size)
            continue;

        GstBuffer *buf = gst_buffer_new_allocate(NULL, size, NULL);
        gst_buffer_fill(buf, 0, vf->getData(), size);

        /* Set explicit timestamps from the SDK frame */
        GST_BUFFER_PTS(buf) = vf->getTimeStampUs() * 1000ULL; /* convert us->ns */
        GST_BUFFER_DURATION(buf) = gst_util_uint64_scale_int(1, GST_SECOND, fps);

        GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), buf);
        /* gst_app_src_push_buffer takes ownership of buf; do not unref here */

        if (ret != GST_FLOW_OK)
        {
            std::cerr << "GStreamer: push-buffer returned " << ret << ", stopping.\n";
            break;
        }

        ++frameCount;
    }

    gst_app_src_end_of_stream(GST_APP_SRC(appsrc));
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    pipe.stop();

    return 0;
}
catch (ob::Error &e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}