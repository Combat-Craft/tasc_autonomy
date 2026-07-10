from pyorbbecsdk import *

# 1.Create a pipeline with default device.
pipeline = Pipeline()
# 2.Get device with pipeline
device = pipeline.get_device()
# 3.Get the sensor list from device.
sensor_list = device.get_sensor_list()
# 4.Enable all available video streams
for sensor in range(len(sensor_list)):
    sensor_type = sensor_list[sensor].get_type()

    print(f"Enabling sensor type: {sensor_type}")
    config.enable_stream(sensor_type)
