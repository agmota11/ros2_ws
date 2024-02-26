// ROS2 MIGRATION
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/qos.hpp"

#include <string>
#include <chrono>

// darknet_ros_msgs
#include "custom_msgs/msg/bounding_box.hpp"
#include "custom_msgs/msg/bounding_boxes.hpp"

// Yolo
#include "cameras_msgs/msg/image_with_bounding_box.hpp"
#include "cameras_msgs/msg/bounding_box.hpp"

// Zed wrapper for depth
#include "sensor_msgs/msg/image.hpp"

// To build the messages to publish
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

#include "custom_msgs/msg/cone.hpp"
#include "custom_msgs/msg/cone_array.hpp"

// Subscribe to camera info
#include <sensor_msgs/msg/camera_info.hpp>

#include <limits>

using namespace std::chrono_literals;
#define MAXLISTSIZE 30

class CamConesNode : public rclcpp::Node
{
public:
    CamConesNode() : Node("publishing_subscriber")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

        // SUBS SIM
        // subDarknet = this->create_subscription<custom_msgs::msg::BoundingBoxes>("/yolov8/bounding_boxes", qos, std::bind(&CamConesNode::darknetMsgCallback, this, _1));
        // subDepth = this->create_subscription<sensor_msgs::msg::Image>("/zed/depth/image_raw", qos, std::bind(&CamConesNode::depthCallback, this, _1));
        subCamInfo = this->create_subscription<sensor_msgs::msg::CameraInfo>("/zed/left/camera_info", qos, std::bind(&CamConesNode::camInfoCallback, this, _1));
        subImages = this->create_subscription<cameras_msgs::msg::ImageWithBoundingBox>("/yolov8/results", qos, std::bind(&CamConesNode::imagesCallback, this, _1));

        // SUBS CAMARAS
        // subDarknet = this->create_subscription<custom_msgs::msg::BoundingBoxes>("/yolov8/bounding_boxes",qos,std::bind(&CamConesNode::darknetMsgCallback, this,_1));
        // subDepth = this->create_subscription<sensor_msgs::msg::Image>("/zed/zed_node/depth/depth_registered",qos,std::bind(&CamConesNode::depthCallback, this,_1));
        // subCamInfo = this->create_subscription<sensor_msgs::msg::CameraInfo>("/zed/zed_node/left/camera_info",qos,std::bind(&CamConesNode::camInfoCallback, this,_1));

        timer_ = this->create_wall_timer(60ms, std::bind(&CamConesNode::timer_callback, this));
        chatter_pub = this->create_publisher<custom_msgs::msg::ConeArray>("/CamConeList", 1);
    }

private:
    struct Cone
    {
        volatile int pixelpos[3] = {-1, -1, -1};
        volatile float pixeldepth = -1;
        std::string cone_color = "null";
    };

    struct Camera
    {
        // Default values (VGA resolution)
        float fx = 340.258;
        float fy = 340.258;
        float cx = 335.276;
        float cy = 182.482;
    };

    Cone coneList[MAXLISTSIZE]; // We declare the list to avoid time wasting allocating memory on runtime.
    Camera CameraInfo;          // Camera settings variable
    int detected_cones = 0;     // For every darknet detected image, detected_cones gets updated with the total number of cones
    float *point;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<custom_msgs::msg::ConeArray>::SharedPtr chatter_pub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subCamInfo;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subDepth;
    rclcpp::Subscription<custom_msgs::msg::BoundingBoxes>::SharedPtr subDarknet;
    rclcpp::Subscription<cameras_msgs::msg::ImageWithBoundingBox>::SharedPtr subImages;

    void imagesCallback(const cameras_msgs::msg::ImageWithBoundingBox::SharedPtr msg)
    {
        msg->bounding_boxes;
        sensor_msgs::msg::Image::SharedPtr depth_image = std::make_shared<sensor_msgs::msg::Image>(msg->depth_image);
        for (unsigned i = 0; i < msg->bounding_boxes.size(); i++)
        { // First we iterate on the total number of detected bounding boxes
            if (i >= MAXLISTSIZE)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Too many cones detected in the image. Returning");
                return;
            }
            coneList[i].pixelpos[0] = (int)(((float)msg->bounding_boxes[i].xmin + (float)msg->bounding_boxes[i].xmax) / 2); // Gets the middle point of the bounding box
            coneList[i].pixelpos[1] = (int)(((float)msg->bounding_boxes[i].ymin + (float)msg->bounding_boxes[i].ymax) / 2);
            coneList[i].cone_color = msg->bounding_boxes[i].class_id;
            detected_cones = i + 1;
        }
        depthCallback(depth_image);
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {

        // Get a pointer to the depth values, casting the data
        // pointer to floating point
        float *depths = (float *)(&msg->data[0]);

        for (int i = 0; i < detected_cones; i++)
        {
            // Now we re-iterate on our list to get the cone depths
            // Linear index of the center pixel
            int centerIdx = coneList[i].pixelpos[0] + msg->width * coneList[i].pixelpos[1];

            // Output the measure
            if (depths[centerIdx] > 0 && depths[centerIdx] < std::numeric_limits<float>::max())
            {
                coneList[i].pixeldepth = depths[centerIdx];
            }
            else
            {
                coneList[i].pixeldepth = 0;
            }
        }
    }

    void camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {

        CameraInfo.fx = msg->k[0];
        CameraInfo.fy = msg->k[4];
        CameraInfo.cx = msg->k[2];
        CameraInfo.cy = msg->k[5];
        /*  Intrinsic camera matrix for the raw (distorted) images.
            [fx  0 cx]
        K = [ 0 fy cy]
            [ 0  0  1]
        Projects 3D points in the camera coordinate frame to 2D pixel
        coordinates using the focal lengths (fx, fy) and principal point
        (cx, cy).
        */
    }

    /* Convert 2D screen coordinates to 3D points */
    float *screenTo3D(int u, int v, float z)
    {
        /* Arguments: u-> row, v-> column, z-> depth */
        /*
        u = fx * (X / Z) + cx
        v = fy * (Y / Z) + cy
        X = Z / fx * (u - cx)
        Y = Z / fy * (v - cy)
        */
        float *point3D = (float *)malloc(3 * sizeof(float));

        point3D[0] = z * (u - CameraInfo.cx) / CameraInfo.fx;
        point3D[1] = z * (v - CameraInfo.cy) / CameraInfo.fy;
        point3D[2] = z;
        return point3D;
    }

    void timer_callback()
    {
        custom_msgs::msg::ConeArray cone_list;
        for (int i = 0; i < detected_cones; i++)
        {
            point = screenTo3D(coneList[i].pixelpos[0], coneList[i].pixelpos[1], coneList[i].pixeldepth);
            if (point)
            {
                custom_msgs::msg::Cone cone;
                cone.x_pos = point[0];
                cone.y_pos = point[1];
                cone.z_pos = point[2];
                cone.color = coneList[i].cone_color;
                if (cone.z_pos > 0)
                {
                    cone_list.cones.push_back(cone);
                }

                free(point);
            }
        }
        cone_list.cone_number = cone_list.cones.size();

        cone_list.header.stamp = rclcpp::Clock{}.now();
        RCLCPP_INFO(this->get_logger(), "Publishing %d cones...", cone_list.cone_number);
        chatter_pub->publish(cone_list);
    };
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CamConesNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}