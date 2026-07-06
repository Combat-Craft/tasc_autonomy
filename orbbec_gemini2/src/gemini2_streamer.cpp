#include <libobsensor/ObSensor.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <iostream>
#include <memory>


class GStreamerPipeline {


    private:
        //orbbec variables
        ob::Pipeline orbbecPipeline_; // Create a pipeline with default device.
        std::shared_ptr<ob::Device> device_ ;
        std::shared_ptr<ob::SensorList> sensorList_ ;
        std::shared_ptr<ob::Config> config_;

        bool startStream_;

        //gstreamer variables

        GstElement *gPipeline;
        GstElement *appsrc;
        GstElement *videoconvert;
        GstElement *encoder;
        GstElement *sink;
        GstBus *bus;
        guint64 frameCount;

        bool running_;

        int width;
        int height;
        int fps;

    public:
        GStreamerPipeline(int argc, char *argv[]){
            // Initialize GStreamer
            gst_init(&argc, &argv);

            // initialize defaults variables
            width = 1280;
            height = 720;
            fps = 30;
            frameCount = 0;
            running_ = false;
        }

        void setWidth(int w) { width = w; }
        void setHeight(int h) { height = h; }
        void setFPS(int f) { fps = f; }

        void handleStreamError(const ob::Error &e) {
            std::cerr << "Function: " << e.getName() << "\nArgs: " << e.getArgs() << "\nMessage: " << e.getMessage() << "\nstatus:" << e.getStatus()
                    << "\nType: " << e.getExceptionType() << std::endl;
        }

        void createOrbbecPipeline() {
            try{
                if(! running_){
                    // 2.Get the device from pipeline.
                    device_ = orbbecPipeline_.getDevice();

                    // 3.Get the sensor list from device.
                    sensorList_ = device_->getSensorList();

                    // 4.Create a config for pipeline.
                    config_ = std::make_shared<ob::Config>();

                    // 5. Set relevant color camera parameters
                    // - Set the Color for automatic exposure.
                    device_->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, true);
                    
                    // 6. Disable other streams and sensors
                    config_->disableStream(OB_STREAM_DEPTH);
                    config_->disableStream(OB_STREAM_IR);
                    config_->disableStream(OB_STREAM_IR_LEFT);
                    config_->disableStream(OB_STREAM_IR_RIGHT);
                    config_->disableStream(OB_STREAM_ACCEL);
                    config_->disableStream(OB_STREAM_GYRO);
                    
                    // 7.Enable color video stream.
                    config_->enableVideoStream(OB_STREAM_COLOR, width, height, fps, OB_FORMAT_YUYV);
                    //config_->enableVideoStream(OB_STREAM_COLOR, width, height, fps, OB_FORMAT_MJPG);

                    startStream_ = false;
                    std::cout << "createOrbbecPipeline() finished with " << width << "x" << height << "@" << fps << std::endl;
                }
                
            }
            catch(ob::Error &e) {
                std::cerr << "createOrbbecPipeline() failed: " << e.what() << std::endl;
                handleStreamError(e);
            }
        }

        void startOrbbecPipeline() {
            if(! running_ && config_ && !startStream_) {
                startStream_ = true;
                orbbecPipeline_.start(config_);
                std::cout << "Started streaming -startOrbbecPipeline() success." << std::endl;
            }
        }

        void stopOrbbecPipeline() {
            if(startStream_) {
                startStream_ = false;
                orbbecPipeline_.stop();
                std::cout << "Stopped streaming - stopOrbbecPipeline() success." << std::endl;
            }
        }

        void createGStreamerPipeline() {

            gPipeline = gst_pipeline_new("orbbec-gstreamer-pipeline");
            appsrc = gst_element_factory_make("appsrc", "orbbec-source");
            videoconvert = gst_element_factory_make("videoconvert", "video-convert");
            encoder = gst_element_factory_make("x264enc", "x264-encoder");
            sink = gst_element_factory_make("srtsink", "srt-output");

            if (!gPipeline || !appsrc || !videoconvert || !encoder || !sink) {
                std::cerr << "Failed to create GStreamer elements." << std::endl;
                return;
            }

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
                "uri", "srt://192.168.1.23:7092?mode=caller",
                "sync", FALSE,
                "latency", 200,
                NULL);

            gst_bin_add_many(GST_BIN(gPipeline), appsrc, videoconvert, encoder, sink, NULL);
            if (!gst_element_link_many(appsrc, videoconvert, encoder, sink, NULL)) {
                std::cerr << "Failed to link GStreamer pipeline." << std::endl;
                gst_object_unref(gPipeline);
                return ;
            }
        } //end of createGStreamerPipeline()

        void startGStreamerPipeline() {
            GstStateChangeReturn stateRet = gst_element_set_state(gPipeline, GST_STATE_PLAYING);
            if (stateRet == GST_STATE_CHANGE_FAILURE) {
                std::cerr << "Unable to set the GStreamer pipeline to the playing state." << std::endl;
                gst_object_unref(gPipeline);
                return;
            }
        }

        void stopGStreamerPipeline() {
            gst_element_set_state(gPipeline, GST_STATE_NULL);
            gst_object_unref(gPipeline);
        }

        void getGStreamerBus() {
            bus = gst_element_get_bus(gPipeline);
        }

        void run() {
            frameCount = 0;
            running_ = true;     
            
            auto frameSet = orbbecPipeline_.waitForFrameset();
            if (!frameSet) {return;}

            auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
            if (!colorFrame) {
                std::cerr << "Frameset missing color frame." << std::endl;
                return;
            }

            auto videoFrame = colorFrame->as<const ob::VideoFrame>();
            if (!videoFrame) {
                std::cerr << "Failed to cast to VideoFrame." << std::endl;
                return;
            }

            const gsize dataSize = static_cast<gsize>(videoFrame->getDataSize());
            GstBuffer *buffer = gst_buffer_new_allocate(NULL, dataSize, NULL);
            if (!buffer) {
                std::cerr << "Failed to allocate GStreamer buffer." << std::endl;
                return;  
            }

            gst_buffer_fill(buffer, 0, videoFrame->getData(), dataSize);

            GST_BUFFER_PTS(buffer) = static_cast<guint64>(videoFrame->getTimeStampUs()) * 1000ULL;
            GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, fps);

            GstFlowReturn flowReturn = gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer);

            if (flowReturn != GST_FLOW_OK) {
                std::cerr << "GStreamer appsrc push failed: " << flowReturn << std::endl;
                return;
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
                        running_ = false;
                        return;
                    case GST_MESSAGE_EOS:
                        std::cout << "GStreamer reached end of stream." << std::endl;
                        running_ = false;
                        return;
                    default:
                        return;
                }
                gst_message_unref(msg);
            }

        }
};

int main(int argc, char **argv)  {
    GStreamerPipeline myStreamer(argc, argv);
    // something run()
    myStreamer.createOrbbecPipeline();
    myStreamer.startOrbbecPipeline();
    myStreamer.createGStreamerPipeline();
    myStreamer.startGStreamerPipeline();
    myStreamer.getGStreamerBus();
    while (true) {
        myStreamer.run();
    }
    //myStreamer.run();

    return 0;
}


