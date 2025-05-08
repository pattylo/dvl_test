#include <ros/ros.h>
#include <std_msgs/String.h>
#include <waterlinked_a50_ros_driver/DVL.h>
#include <waterlinked_a50_ros_driver/DVLBeam.h>

#include <nlohmann/json.hpp>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

using json = nlohmann::json;

int sock_fd;
std::string TCP_IP;
int TCP_PORT;
bool do_log_raw_data;

std::string oldJson = "";

void connect()
{
    struct sockaddr_in serv_addr;
    sock_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd < 0)
    {
        ROS_ERROR("Socket creation error");
        ros::shutdown();
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(TCP_PORT);

    if (inet_pton(AF_INET, TCP_IP.c_str(), &serv_addr.sin_addr) <= 0)
    {
        ROS_ERROR("Invalid address / Address not supported");
        ros::shutdown();
    }

    while (connect(sock_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        ROS_ERROR("No route to host, DVL might be booting?");
        ros::Duration(1.0).sleep();
    }

    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
}

std::string getData()
{
    char c;
    std::string raw_data;

    while (raw_data.find('\n') == std::string::npos)
    {
        ssize_t len = recv(sock_fd, &c, 1, 0);
        if (len == 0)
        {
            ROS_ERROR("Socket closed by the DVL, reopening");
            connect();
            continue;
        }
        else if (len < 0)
        {
            ROS_ERROR("Lost connection with the DVL, reinitiating the connection");
            connect();
            continue;
        }
        raw_data += c;
    }

    raw_data = oldJson + raw_data;
    size_t newline_pos = raw_data.find('\n');
    oldJson = raw_data.substr(newline_pos + 1);
    return raw_data.substr(0, newline_pos);
}

void publisher()
{
    ros::NodeHandle nh;
    ros::Publisher pub_raw = nh.advertise<std_msgs::String>("dvl/json_data", 10);
    ros::Publisher pub = nh.advertise<waterlinked_a50_ros_driver::DVL>("dvl/data", 10);
    ros::Rate rate(10);

    while (ros::ok())
    {
        std::string raw_data = getData();

        json data;
        try
        {
            data = json::parse(raw_data);
        }
        catch (json::parse_error &e)
        {
            ROS_ERROR("JSON parse error: %s", e.what());
            continue;
        }

        std_msgs::String msg_raw;
        msg_raw.data = raw_data;

        if (do_log_raw_data)
        {
            ROS_INFO("%s", raw_data.c_str());
            pub_raw.publish(msg_raw);
            if (data["type"] != "velocity") continue;
        }
        else
        {
            if (data["type"] != "velocity") continue;
            pub_raw.publish(msg_raw);
        }

        waterlinked_a50_ros_driver::DVL dvl_msg;
        dvl_msg.header.stamp = ros::Time::now();
        dvl_msg.header.frame_id = "dvl_link";
        dvl_msg.time = data["time"];
        dvl_msg.velocity.x = data["vx"];
        dvl_msg.velocity.y = data["vy"];
        dvl_msg.velocity.z = data["vz"];
        dvl_msg.fom = data["fom"];
        dvl_msg.altitude = data["altitude"];
        dvl_msg.velocity_valid = data["velocity_valid"];
        dvl_msg.status = data["status"];
        dvl_msg.form = data["format"];

        for (int i = 0; i < 4; ++i)
        {
            waterlinked_a50_ros_driver::DVLBeam beam;
            beam.id = data["transducers"][i]["id"];
            beam.velocity = data["transducers"][i]["velocity"];
            beam.distance = data["transducers"][i]["distance"];
            beam.rssi = data["transducers"][i]["rssi"];
            beam.nsd = data["transducers"][i]["nsd"];
            beam.valid = data["transducers"][i]["beam_valid"];
            dvl_msg.beams.push_back(beam);
        }

        pub.publish(dvl_msg);
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "a50_pub");
    ros::NodeHandle nh("~");

    nh.param<std::string>("ip", TCP_IP, "10.42.0.186");
    nh.param<int>("port", TCP_PORT, 16171);
    nh.param<bool>("do_log_raw_data", do_log_raw_data, false);

    connect();
    publisher();

    close(sock_fd);
    return 0;
}
