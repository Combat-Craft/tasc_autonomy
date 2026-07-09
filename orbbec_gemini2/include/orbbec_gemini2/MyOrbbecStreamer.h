#include <libobsensor/ObSensor.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <iostream>
#include <memory>
#include <string>

using namespace std;

class MyOrbbecStreamer {
  private:
    // color stream control variables
    int width;
    int height;
    int fps;
    int bitrate; // in kbps
    string srtURI;
    /*
    int brightness;
    int exposure; //(min:0, max:33000, step:1)
    int exposure_auto; // true = 1
    int gain; //(min:1, max:255, step:2)
    //etc ....
    */

    GstElement *pipeline;
    GstBus *bus;

  public:
    // === METHODS ====================================================================================
    // constructor 
    MyOrbbecStreamer(int argc, char *argv[]);
    void setWidth   (int w);
    void setHeight  (int h);
    void setFPS     (int f);  
    void setbitrate (int bitr);
    void setURI(string s);
    //void set ();
    //void set ();
    //void set ();
    //void set ();
    
    void runStream();
    void stopStream();
    void pauseStream();
    void restartStream();
};




