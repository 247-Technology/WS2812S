#include <ros/ros.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <iostream>
#include <thread>
#include <mutex>

const int NUM_SENSORS = 3;
const std::string SENSOR_NAMES[NUM_SENSORS] = {"sensor_0", "sensor_1", "sensor_2"};

ros::Publisher distance_pubs[NUM_SENSORS];
ros::Publisher led_control_pub;
serial::Serial serial_port;
std::mutex serial_mutex;

void readAndPublishSensorData()
{
    while (ros::ok())
    {
        std::unique_lock<std::mutex> lock(serial_mutex);
        std::string data = serial_port.readline();
        lock.unlock();

        std::vector<std::string> sensor_data;
        size_t pos = 0;

        while ((pos = data.find("\t")) != std::string::npos)
        {
            sensor_data.push_back(data.substr(0, pos));
            data.erase(0, pos + 1);
        }

        for (size_t i = 0; i < sensor_data.size(); ++i)
        {
            std::string sensor_info = sensor_data[i];

            size_t sensor_pos = sensor_info.find("Sensor");
            if (sensor_pos != std::string::npos)
            {
                int sensor_id = sensor_info[sensor_pos + 7] - '0';
                std::string sensor_distance = sensor_info.substr(sensor_info.find("Distance:") + 10);

                if (sensor_id < NUM_SENSORS)
                {
                    std_msgs::String msg;
                    msg.data = sensor_distance;
                    distance_pubs[sensor_id].publish(msg);
                }
            }
        }
    }
}

void userInputThread()
{
    while (ros::ok())
    {
        std::string led_command;
        std::cout << "Enter LED command: ";
        std::cin >> led_command;
        std::unique_lock<std::mutex> lock(serial_mutex);
        serial_port.write(led_command);
        lock.unlock();
        
        std_msgs::String msg;
        msg.data = led_command;
        led_control_pub.publish(msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_to_ros");
    ros::NodeHandle nh;

    try
    {
        serial_port.setPort("/dev/ttyUSB0");
        serial_port.setBaudrate(115200);
        serial_port.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("DON'T OPEN SERIAL PORT. ERROR: " << e.what());
        return -1;
    }

    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        distance_pubs[i] = nh.advertise<std_msgs::String>(SENSOR_NAMES[i], 10);
    }

    led_control_pub = nh.advertise<std_msgs::String>("led_control", 10);

    std::thread reading_thread(readAndPublishSensorData);
    std::thread input_thread(userInputThread);

    ros::spin();

    serial_port.close();

    return 0;
}
