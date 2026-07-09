#include <libobsensor/ObSensor.hpp>


#include <gstreamermm.h>
#include <gstreamermm/appsrc.h>
#include <gstreamermm/value.h>
#include <glibmm/main.h>

#include <iostream>
#include <memory>
#include <string>

using namespace std;

class MyOrbbecStreamer {
  public:
    // GStreamer parameters
    Glib::RefPtr<Glib::MainLoop> main_loop;
    
    Glib::RefPtr<Gst::Pipeline> gPipeline;
    //Glib::RefPtr<Gst::Element> source;
    Glib::RefPtr<Gst::AppSrc> source;
    Glib::RefPtr<Gst::Caps> caps;
    Glib::RefPtr<Gst::Element> encoder;
    Glib::RefPtr<Gst::Element> convert;
    Glib::RefPtr<Gst::Element> sink;
    
    // color stream control variables
    int width;
    int height;
    int fps;
    int bitrate; // in kbps
    string srtURI;
    
    //orbbec variables
    ob::Pipeline orbbecPipeline_;
    std::shared_ptr<ob::Device> device_ ;
    std::shared_ptr<ob::SensorList> sensorList_ ;
    std::shared_ptr<ob::Config> config_;
    
    bool orbbec_running_ = false;
    guint64 frameCount;
  
    
    //constructor
    MyOrbbecStreamer(int argc, char *argv[]) {
        cout << "MyOrbbecStreamer() contructor called" << std::endl;
        Gst::init(argc, argv);
        
        // initialize defaults variables
        width = 1280;
        height = 720;
        fps = 30;
        bitrate = 512;
        srtURI = "srt://127.0.0.1:7092?mode=caller";
    }
   
    void startOrbbecPipeline(){
      // Create a Context
      //ob::Context ctx;
      //cout << "made ctx" << std::endl;
      // 1.Find all connected devices and the 1st/only
      //std::shared_ptr<ob::DeviceList> devices = ctx.queryDeviceList();
      //auto device_  = devices->getDevice(0);
      //cout << "made device" << std::endl;
      
      // 2.Create a pipeline with default device_.
      //auto orbbecPipeline_ = std::make_shared<ob::Pipeline>(device_);
      //cout << "made orbbecpipeline" << std::endl;
      
      ob::Pipeline orbbecPipeline_;
      
      // 2.Get the device from pipeline.
      device_ = orbbecPipeline_.getDevice();
      // 3.Get the sensor list from device_.
      sensorList_ = device_->getSensorList();
      cout << "made sensorlist" << std::endl;
      // 4.Create a config_ for pipeline.
      config_ = std::make_shared<ob::Config>();
      cout << "made config" << std::endl;
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
      const int width = 1280, height = 720, fps = 30;
      config_->enableVideoStream(OB_STREAM_COLOR, width, height, fps, OB_FORMAT_YUYV);
      
      cout << "set config" << std::endl;
      
      std::cout << "trying to start orbbec pipeline" << std::endl;
      
      frameCount = 0;
      
      // Start the pipeline with config.
      orbbecPipeline_.start(config_);
      
      createGstreamerPipeline();
      startGstreamerPipeline();
      
      orbbec_running_ = true;
      
      std::cout << "orbbec pipeline is strating to playing" << std::endl;
      
      while(orbbec_running_){
      
        auto frameSet = orbbecPipeline_.waitForFrameset();
        if (!frameSet) {
            continue;
        }
        
        std::cout << "=Frameset grabbed" << std::endl;
        
        auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
        if (!colorFrame) {
            std::cerr << "Frameset missing color frame." << std::endl;
            continue;
        } 
        else{
          std::cout << "got colorFrame via getFrame()." << std::endl;
          
          auto videoFrame = colorFrame->as<ob::VideoFrame>();
          if (!videoFrame) {
              std::cerr << "Failed to cast to VideoFrame." << std::endl;
              continue;
          }
          else{std::cout << "got VideoFrame." << std::endl;}
          
          //uint32_t        width  = videoFrame->getWidth();
          //uint32_t        height = videoFrame->getHeight();
          const gsize dataSize = videoFrame->getDataSize();
          const uint8_t *frameData   = reinterpret_cast<const uint8_t *>(videoFrame->getData());
          
          //create the buffer
          auto buffer = Gst::Buffer::create(dataSize); //static_cast<guint>(videoFrame->getDataSize())
          if (!buffer) {
              std::cerr << "Failed to create GStreamer buffer." << std::endl;
              return;
          }
          else{
              std::cout << "buffer made." << std::endl;
          }
          
          Gst::MapInfo mapinfo;
          buffer->map(mapinfo, Gst::MAP_WRITE);
          std::memcpy(mapinfo.get_data(), frameData, dataSize);
          // map is unmapped when 'map' goes out of scope
          
          std::cout << "buffer filleds." << std::endl;
          
          
          // timestamp
          const guint64 pts = static_cast<guint64>(videoFrame->getTimeStampUs()) * 1000ULL;
          buffer->set_pts(pts);
          buffer->set_duration( (GST_SECOND + fps / 2) / fps );
          
          source->push_buffer(buffer);
          
          frameCount++;
          if ((frameCount % 30) == 0) {
              std::cout << "Pushed " << frameCount << " frames to GStreamer." << std::endl;
          }
        }
      }
      
      
      
    }// END createOrbbecPipeline()
    
    
    void stopOrbbecPipeline() {
      orbbecPipeline_.stop();
    }
    
    
    void createGstreamerPipeline(){
      // Create the empty pipeline
      gPipeline = Gst::Pipeline::create("orbbec-appsrc-pipeline");
    
      // Create some elements
      //source = Gst::ElementFactory::create_element("videotestsrc", "source");
      source = Gst::AppSrc::create("orbbec-source");
      convert = Gst::ElementFactory::create_element("videoconvert", "convert");
      encoder = Gst::ElementFactory::create_element("x264enc", "encoder");
      sink = Gst::ElementFactory::create_element("autovideosink", "sink"); // "srtsink"
         
      if (!gPipeline || !source  || !convert || !encoder ||  !sink) {
        std::cerr << "Failed to create GStreamer elements." << std::endl;
        return;
      } 

      // Add elements to a pipeline
      try {
        gPipeline->add(source)->add(convert)->add(sink);
        //gPipeline->add(source)->add(convert)->add(encoder)->add(sink);
      }
      catch (const std::runtime_error& ex){
        std::cerr << "Exception while adding: " << ex.what() << std::endl;
        return;
      }

      // Link elements
      try {
        source->link(convert)->link(sink);
      }
      catch (const std::runtime_error& ex){
        std::cerr << "Exception while linking: " << ex.what() << std::endl;
        return;
      }
      
      //Create caps
      //Glib::Value<Gst::Fourcc> value;
      //value.init(Glib::Value<Gst::Fourcc>::value_type());
      //value.set(Gst::Fourcc('Y', 'U', 'U', '2'));
      caps = Gst::Caps::create_simple("video/x-raw",
                                      "format", GST_VIDEO_FORMAT_YUY2,
                                      "framerate", Gst::Fraction(fps, 1),
                                      "width", width,
                                      "height", height);

      // Set source properties
      source->set_property("caps", caps);
      source->set_property("format", Gst::Format::FORMAT_TIME );//GST_FORMAT_TIME as timestamppeds
      source->set_property("is-live", true);
      source->set_property("block", true);
      source->set_property("stream-type", 0); // GST_APP_STREAM_TYPE_STREAM (0) – No seeking is supported in the stream, such as a live stream.
      
      // set encoder
      encoder->set_property("bitrate", bitrate);
      encoder->set_property("tune", 0x00000004);  //zerolatency
      encoder->set_property("pass", 17);          // pass1 i.e. VBR
      encoder->set_property("speed-preset", 1);   // ultrafast, for the lowest CPU
      
      // Set SRT sink
      /*
      sink->set_property("uri", srtURI);
      sink->set_property("sync", false);
      sink->set_property("latency", 200);
      sink->set_property("keep-listening", true);
      */
    
    } //END createGstreamerPipeline
    
    void startGstreamerPipeline(){
      // Start pipeline
      gPipeline->set_state(Gst::STATE_PLAYING);
      std::cout << "gstramer is playing" << std::endl;

      //main_loop = Glib::MainLoop::create();
      //main_loop->run();
    }
    
    void stopGstreamerPipeline(){
      // Stop playing the pipeline
      gPipeline->set_state(Gst::STATE_NULL);
    }
    
    
};


int main(int argc, char *argv[]){
  MyOrbbecStreamer streamer(argc, argv) ;
  // call any setX() before any createXPipeline() methods
  //streamer.createGstreamerPipeline();
  //streamer.startGstreamerPipeline();
  streamer.startOrbbecPipeline();


  //streamer.stopGstreamerPipeline();
  //streamer.runOrbbecGstreamer() ;
  return 0;
}
