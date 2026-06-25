#include <libobsensor/ObSensor.hpp>

int main()
{
    try
    {
        ob::Pipeline pipe;
        auto config = std::make_shared<ob::Config>();

        config->enableVideoStream(
            OB_STREAM_COLOR, 1280, 720, 30, OB_FORMAT_MJPG);

        std::cout << "Starting color-only stream: 1280x720 @ 30 FPS" << std::endl;

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

        ob_smpl::CVWindow win("Color", 1280, 720);
        win.setShowSyncTimeInfo(true);

        while (win.run())
        {
            auto frameset = pipe.waitForFrameset();
            if (!frameset)
            {
                std::cerr << "No frames received." << std::endl;
                continue;
            }

            auto colorFrame = frameset->colorFrame();
            if (!colorFrame)
            {
                std::cerr << "Frameset had no color frame." << std::endl;
                continue;
            }

            win.pushFramesToView(colorFrame);
        }

        pipe.stop();
        return 0;
    }
    catch (const ob::Error &e)
    {
        std::cerr << "Orbbec SDK error: " << e.what() << std::endl;
        return 1;
    }
}
