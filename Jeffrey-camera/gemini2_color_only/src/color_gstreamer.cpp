#include <libobsensor/ObSensor.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>

#include "utils.hpp"
#include "utils_opencv.hpp"

int main(int argc, char **argv)
{
    try
    {

        ob::Pipeline pipe;
        auto config = std::make_shared<ob::Config>();

        config->enableVideoStream(
            OB_STREAM_COLOR, 1280, 720, 30, OB_FORMAT_MJPG);

        std::cout << "Starting Gemini2 color stream for GStreamer" << std::endl;

        pipe.start(config);

        try
        {
            auto profiles = pipe.getStreamProfileList(OB_SENSOR_COLOR);
            for (uint32_t i = 0; i < profiles->getCount(); ++i)
            {
                auto sp = profiles->getProfile(i);
                auto vp = std::dynamic_pointer_cast<ob::VideoStreamProfile>(sp);
                if (vp)
                {
                    std::cout << "Profile " << i << ": " << vp->getWidth() << "x" << vp->getHeight()
                              << " @ " << vp->getFps() << " FPS format=" << static_cast<int>(vp->getFormat()) << std::endl;
                }
            }
        }
        catch (...)
        {
            std::cerr << "Failed to get stream profiles." << std::endl;
        }

        std::cout << "GStreamer viewer running. Press Ctrl+C to exit." << std::endl;

        ob_smpl::CVWindow window("GStreamer Preview", 1280, 720);
        window.setShowSyncTimeInfo(true);

        while (window.run())
        {
            auto frameset = pipe.waitForFrameset();
            if (!frameset)
            {
                std::cerr << "No frames received." << std::endl;
                continue;
            }

            auto colorFrame = frameset->getFrame(OB_FRAME_COLOR);
            if (!colorFrame)
            {
                std::cerr << "Frameset had no color frame." << std::endl;
                continue;
            }

            auto videoFrame = colorFrame->as<const ob::VideoFrame>();
            if (!videoFrame)
            {
                std::cerr << "Failed to cast to VideoFrame." << std::endl;
                continue;
            }

            window.pushFramesToView(colorFrame);
        }

        pipe.stop();
        cv::destroyAllWindows();
        return 0;
    }
    catch (const ob::Error &e)
    {
        std::cerr << "Orbbec SDK error: " << e.what() << std::endl;
        return 1;
    }
}