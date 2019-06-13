// posefusion-server.cpp
// Subscribes to mqtt topic that contains pose estimation data and attempts
// to combine the data into an accurate 3D pose of people

#include "mqtt/client.h"

const std::string SERVER_ADDR   = "127.0.0.1:1883";
const std::string CLIENT_ID     = "im-a-big-pc";
const std::string TOPIC         = "lambda-1-pose";
const std::string EXIT_MSG      = "EXIT";

const int QOS = 0;

int main(int argc, char *argv[])
{
    std::shared_ptr<const mqtt::message> msg;

    std::cout << "Initialzing MQTT Client...";
    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(20);
    connOpts.set_clean_session(true);
    mqtt::client client(SERVER_ADDR, CLIENT_ID);
    std::cout << "OK" << std::endl;

    try {
        std::cout << "Connecting to MQTT Server..." << std::flush;
        client.connect(connOpts);
        client.subscribe(TOPIC, QOS);
        std::cout << "OK" << std::endl;

        while (1) {
            if (client.try_consume_message(&msg)) {
                if(msg->to_string().compare(EXIT_MSG))
                    std::cout << msg->to_string() << std::endl;
                else
                    break;
            }
        }

        std::cout << "Disconnecting MQTT...";
        client.disconnect();
        std::cout << "OK" << std::endl;
    }
    catch (const mqtt::exception& exc) {
        std::cerr << exc.what() << std::endl;
        return 1;
    }

    std::cout << "Exiting" << std::endl;
    return 0;
}