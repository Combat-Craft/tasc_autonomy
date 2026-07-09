#include <libobsensor/ObSensor.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <iostream>
#include <memory>

int main(int argc, char **argv) {

    // Create a pipeline with default device.
    ob::Pipeline pipe;
    
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
    
    //if (config->isStreamEnabled(OB_SENSOR_IR) ){  
    //    std::cout << "\nIR sensor enabled";
    //}

    // Start the pipeline 

    pipe.start(config);

    while(true) {
        // 5.Wait for up to 100ms for a frameset in blocking mode.
        auto frameSet = pipe.waitForFrameset(100);
        if(frameSet == nullptr) {
            continue;
        }
      
         
    }
    // Initialize GStreamer 
        gst_init(&argc, &argv);

        GstElement *pipeline = gst_pipeline_new("orbbec-test-pipeline");
        GstElement *appsrc = gst_element_factory_make("v4l2src", "orbbec-source");
        GstElement *videoconvert = gst_element_factory_make("videoconvert", "video-convert");
        GstElement *sink = gst_element_factory_make("autovideosink", "sink");

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
                     "device", "/dev/orbbec_color_cam",
                     NULL);
        gst_caps_unref(caps);


        g_object_set(sink,
            "sync", FALSE,
            NULL);

        gst_bin_add_many(GST_BIN(pipeline), appsrc, videoconvert, sink, NULL);
        if (!gst_element_link_many(appsrc, videoconvert, sink, NULL)) {
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
    // 6.Stop the pipeline
    pipe.stop();


    return 0;
}

