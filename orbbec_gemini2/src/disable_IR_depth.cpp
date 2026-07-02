#include <libobsensor/ObSensor.hpp>
#include <iostream>
#include <memory>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

int main()
{
    // Create a pipeline with default device.
    ob::Pipeline pipeline;
    
    // 2.Get the device from pipeline.
    std::shared_ptr<ob::Device> device = pipe.getDevice();

    // 3.Get the sensor list from device.
    std::shared_ptr<ob::SensorList> sensorList = device->getSensorList();

    // 4.Create a config for pipeline.
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    
    // 5. Set relevant color camera parameters
    // - Set the Color for automatic exposure.
    device->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, true);

    // 6. Disable other streams and sensors
    config->disableStream(OB_STREAM_DEPTH);
    config->disableStream(OB_STREAM_IR);
    config->disableStream(OB_STREAM_IR_LEFT);
    config->disableStream(OB_STREAM_IR_RIGHT);
    config->disableStream(OB_STREAM_ACCEL);
    config->disableStream(OB_STREAM_GYRO);
    
    // 7.Enable color video stream.
    const int width = 1280, height = 720, fps = 30;
    //config->enableVideoStream(OB_STREAM_COLOR, width, height, fps, OB_FORMAT_MJPG);
    config->enableVideoStream(OB_STREAM_COLOR, width, height, fps, OB_FORMAT_YUYV);
    
    if (config->isStreamEnabled(OB_SENSOR_IR) ){  
        std::cout << "\nIR sensor enabled";
    }

    /* Initialize GStreamer */
    gst_init(&argc, &argv);

    GstElement *pipeline = gst_pipeline_new("orbbec-appsrc-pipeline");
    GstElement *appsrc = gst_element_factory_make("appsrc", "orbbec-source");
    GstElement *videoconvert = gst_element_factory_make("videoconvert", "video-convert");
    GstElement *encoder = gst_element_factory_make("x264enc", "x264-encoder");
    GstElement *sink = gst_element_factory_make("srtsink", "srt-output");

    if (!pipeline || !appsrc  || !videoconvert || !sink) {
        std::cerr << "Failed to create GStreamer elements." << std::endl;
        return EXIT_FAILURE;
    }

    // gst-launch-1.0 v4l2src device=/dev/orbbec_color_cam
    // ! video/x-raw, format=YUY2, width=1280, height=4720, framerate=30/1
    // ! videoconvert
    // ! x264enc pass=pass1 bitrate=500 tune=zerolatency speed-preset=ultrafast
    // ! srtsink uri="srt://192.168.1.23:7092?mode=caller" latency=200 sync=false

    GstCaps *caps = gst_caps_new_simple(
        "video/x-raw",
        "format", G_TYPE_STRING, "YUY2",
        "width", G_TYPE_INT, width,
        "height", G_TYPE_INT, height,
        "framerate", GST_TYPE_FRACTION, fps, 1,
        NULL);

    g_object_set(appsrc,
                 "caps", caps,
                 "format", GST_FORMAT_TIME,
                 "is-live", TRUE,
                 "block", TRUE,
                 "stream-type", GST_APP_STREAM_TYPE_STREAM,
                 NULL);
    gst_caps_unref(caps);

    g_object_set(encoder,
        "bitrate", 1000,         // kbps
        "tune", 0x00000004,     // zerolatency
        "pass", 17,             // pass1 i.e. VBR
        "speed-preset", 1,      // ultrafast
        NULL);

    g_object_set(sink,
        "uri", "srt://127.0.0.1:7092?mode=caller",
        "sync", FALSE,
        "latency", 200,
        NULL);

    gst_bin_add_many(GST_BIN(pipeline), appsrc, videoconvert, encoder, sink, NULL);
    if (!gst_element_link_many(appsrc, videoconvert, encoder, sink, NULL)) {
        std::cerr << "Failed to link GStreamer pipeline." << std::endl;
        gst_object_unref(pipeline);
        return EXIT_FAILURE;
    }

    GstStateChangeReturn stateRet = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (stateRet == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "Unable to set the GStreamer pipeline to the playing state." << std::endl;
        gst_object_unref(pipeline);
        return EXIT_FAILURE;
    }

    GstBus *bus = gst_element_get_bus(pipeline);
    guint64 frameCount = 0;
    bool running = true;

    while (running) {
        auto frameSet = pipe.waitForFrameset();
        if (!frameSet) {
            continue;
        }

        auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
        if (!colorFrame) {
            std::cerr << "Frameset missing color frame." << std::endl;
            continue;
        }

        auto videoFrame = colorFrame->as<const ob::VideoFrame>();
        if (!videoFrame) {
            std::cerr << "Failed to cast to VideoFrame." << std::endl;
            continue;
        }

        const gsize dataSize = static_cast<gsize>(videoFrame->getDataSize());
        GstBuffer *buffer = gst_buffer_new_allocate(NULL, dataSize, NULL);
        if (!buffer) {
            std::cerr << "Failed to allocate GStreamer buffer." << std::endl;
            break;
        }

        gst_buffer_fill(buffer, 0, videoFrame->getData(), dataSize);

        guint64 pts = static_cast<guint64>(videoFrame->getTimeStampUs()) * 1000ULL;
        GST_BUFFER_PTS(buffer) = pts;
        GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, fps);

        GstFlowReturn flowReturn = gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer);

        if (flowReturn != GST_FLOW_OK) {
            std::cerr << "GStreamer appsrc push failed: " << flowReturn << std::endl;
            break;
        }

        frameCount++;
        if ((frameCount % 30) == 0) {
            std::cout << "Pushed " << frameCount << " frames to GStreamer." << std::endl;
        }

        GstMessage *msg = gst_bus_pop_filtered(bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
        if (msg) {
            GError *err = nullptr;
            gchar *debugInfo = nullptr;

            switch (GST_MESSAGE_TYPE(msg)) {
                case GST_MESSAGE_ERROR:
                    gst_message_parse_error(msg, &err, &debugInfo);
                    std::cerr << "GStreamer error: " << err->message << std::endl;
                    g_clear_error(&err);
                    g_free(debugInfo);
                    running = false;
                    break;
                case GST_MESSAGE_EOS:
                    std::cout << "GStreamer reached end of stream." << std::endl;
                    running = false;
                    break;
                default:
                    break;
            }
            gst_message_unref(msg);
        }
    }

    gst_app_src_end_of_stream(GST_APP_SRC(appsrc));
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(bus);
    gst_object_unref(pipeline);
    

    auto inputWatchThread = std::thread([]{
        while(true) {
            std::string cmd;
            std::cout << "\nInput quit to close:  ";
            std::getline(std::cin, cmd);
            if(cmd == "quit" ) {
                break;
            }
        }
    });
    inputWatchThread.detach();
    pipe.stop();


    return 0;
}

catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\nstatus:" << e.getStatus()
              << "\ntype:" << e.getExceptionType() << std::endl;
    std::cout << "\nPress any key to exit.";
    //ob_smpl::waitForKeyPressed();
    exit(EXIT_FAILURE);
} 

    return 0;
}