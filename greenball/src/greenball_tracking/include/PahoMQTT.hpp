#include <iostream>
#include <string>
#include <mqtt/async_client.h>

class PahoMQTT {
private:
    std::string address;
    std::string clientId;
    mqtt::async_client client;

public:
    PahoMQTT(const std::string& addr, const std::string& id)
        : address(addr), clientId(id), client(addr, id) {}

    void connect() {
        mqtt::connect_options connOpts;
        connOpts.set_keep_alive_interval(20);
        connOpts.set_clean_session(true);
        std::cout << "Connecting to the MQTT server..." << std::flush;
        mqtt::token_ptr conntok = client.connect(connOpts);
        conntok->wait();
        std::cout << "OK\n" << std::endl;
    }

    void disconnect() {
        std::cout << "Disconnecting from the MQTT server..." << std::flush;
        mqtt::token_ptr disconntok = client.disconnect();
        disconntok->wait();
        std::cout << "OK\n" << std::endl;
    }

    void setColor(unsigned char r, unsigned char g, unsigned char b) {
        std::string payload = {r, g, b};
        mqtt::message_ptr pubmsg = mqtt::make_message("face_light/color", payload);
        pubmsg->set_qos(1);
        std::cout << "Publishing message..." << std::flush;
        mqtt::token_ptr pubtok = client.publish(pubmsg);
        pubtok->wait();
        std::cout << "OK\n" << std::endl;
    }
};
