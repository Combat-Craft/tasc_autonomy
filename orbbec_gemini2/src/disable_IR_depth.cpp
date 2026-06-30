#include <libobsensor/ObSensor.hpp>
#include <iostream>

int main()
{
    try
    {
        ob::Pipeline pipeline;
        auto config = std::make_shared<ob::Config>();

        config->enableVideoStream(
            OB_STREAM_COLOR,
            1280,
            720,
            30,
            OB_FORMAT_MJPG);

        config->disableStream(OB_STREAM_DEPTH);
        config->disableStream(OB_STREAM_IR);
        config->disableStream(OB_STREAM_IR_LEFT);
        config->disableStream(OB_STREAM_IR_RIGHT);
        config->disableStream(OB_STREAM_ACCEL);
        config->disableStream(OB_STREAM_GYRO);

        pipeline.start(config);

        std::cout << "Orbbec running in COLOR ONLY mode (1280x720 @ 30FPS)" << std::endl;

        while (true)
        {
            pipeline.waitForFrameset();
        }

        pipeline.stop();
    }
    catch (ob::Error &e)
    {
        std::cerr << "Orbbec Error: "
                  << e.getMessage()
                  << std::endl;
        return -1;
    }

    return 0;
}