#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "motion_msgs/msg/motion_ctrl.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "data_msgs/msg/velocity_data.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>
#include <utility> 

using namespace std;

class DiabloSquarePathNode : public rclcpp::Node
{
public:
    DiabloSquarePathNode() : Node("path_follower")
    {
        RCLCPP_INFO(this->get_logger(), "Node Started!");

        diablo_pose_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&DiabloSquarePathNode::poseCallback, this, std::placeholders::_1));

        motion_publisher_ = this->create_publisher<motion_msgs::msg::MotionCtrl>(
            "diablo/MotionCmd", 10);
	
	velocity_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "diablo/velocity", 10);
	
	data_publisher = this->create_publisher<data_msgs::msg::VelocityData>(
            "/VelocityData", 10);

        // Initialize control parameters
        Kv = 0.2;       // Forward gain factor
        Kw = 2.0;      // Rotation gain factor
        
        initial_x = 0.0;
        initial_y = 0.0;
        x_c = 0.0;
        y_c = 0.0;
        yaw_c = 0.0;
        num_points = 1000;
	shortest_d = 0;
        firstMsg = true;

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&DiabloSquarePathNode::start, this));
    }

private:

    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract pitch, yaw, and roll from the quaternion orientation
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        //create a variable to store the current yaw reading
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        double yaw = atan2(siny_cosp, cosy_cosp);

        //Extract position readings
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        x_c = x;
        y_c = y;
        yaw_c = yaw;
        Dv_y = 0; 
        Dv_x = 0;

        if (firstMsg)
        {
            initial_x = x;
            initial_y = y;
            firstMsg = false;
        }

    }

    void start() {
        
        bool initialized = false;
        if (!initialized) {
            // Make the robot stand up
            motion_msgs::msg::MotionCtrl cmd;
            cmd.mode_mark = true;
            cmd.value.up = 1.0;
            cmd.mode.stand_mode = true;
            motion_publisher_->publish(cmd);

            path_vector = create_circle_vector(0, 0, 0.5, num_points);
            initialized = true;
        }

        pair<double, double> desired_velocity = calculateDesiredVelocity(path_vector, x_c, y_c);
        Dv_x = desired_velocity.first;
        Dv_y = desired_velocity.second;
        
        move_to (Dv_x, Dv_y, x_c, y_c);

    }

    std::vector<std::pair<double, double>> create_circle_vector(double x_center, double y_center, double radius, int num_points) {
        std::vector<std::pair<double, double>> circle_vector;
        circle_vector.reserve(num_points); // Reserve space for num_points elements
        
        for (int i = 0; i < num_points; ++i) {
            double x = x_center + radius * 1 * std::cos(2 * M_PI * i / num_points);
            double y = y_center + radius * 0.5 *  std::sin(2 * M_PI * i / num_points);
            circle_vector.emplace_back(x, y);
        }
        
        return circle_vector;
    }

        pair<double, double> calculateDesiredVelocity(const vector<pair<double, double>>& path_vector, double x_c, double y_c) {
            //Calculate the Shortest distance to the path
            shortest_d = numeric_limits<double>::max();
            double distance = 0;
            size_t index_s = 0;
            
            for (size_t i = 0; i < path_vector.size(); ++i) {
                distance = sqrt(pow(path_vector[i].first - x_c, 2) + pow(path_vector[i].second - y_c, 2));
                if (distance < shortest_d) {
                    shortest_d = distance;
                    index_s = i;
                }
            }

            //print the distance
            RCLCPP_INFO(this->get_logger(), "Shortest Distance to the curve: %.2f \n",shortest_d);

            //calculate the Normal
            double N_x = 0;
            double N_y = 0;
            if (shortest_d != 0) {
            N_x = (path_vector[index_s].first - x_c) / (shortest_d);
            N_y = (path_vector[index_s].second - y_c) / (shortest_d);
	    }
            size_t next_index = (index_s + 1) % path_vector.size();
            // Calculate the magnitude of the vector difference
            double mag = sqrt(pow(path_vector[next_index].first - path_vector[index_s].first, 2) +
                                pow(path_vector[next_index].second - path_vector[index_s].second, 2));

            // Calculate the unit vector components in the tangent direction
            double T_x = (path_vector[next_index].first - path_vector[index_s].first) / mag;
            double T_y = (path_vector[next_index].second - path_vector[index_s].second) / mag;

            // Calculate the linear combination weights
            double k_0 = 0.1;
            double alpha = shortest_d / sqrt(pow(shortest_d, 2) + pow(k_0, 2));
            double beta = k_0 / sqrt(pow(shortest_d, 2) + pow(k_0, 2));

            // Calculate the desired velocity components
            double Dv_x = (alpha * N_x) + (beta * T_x);
            double Dv_y = (alpha * N_y) + (beta * T_y);
            
            RCLCPP_INFO(this->get_logger(), "Desired V_x: %.2f \n",Dv_x);
            RCLCPP_INFO(this->get_logger(), "Desired V_x: %.2f \n",Dv_y);
            
            return {Dv_x, Dv_y};
        }

    void move_to (double Dv_x, double Dv_y,  double xc, double yc) {
        double forward_speed_ =   Kv * ((cos(yaw_c) * Dv_x)+(sin(yaw_c) * Dv_y));
        double rotation_speed_ =  Kw * ((cos(yaw_c) * Dv_y)-(sin(yaw_c) * Dv_x));
        
        RCLCPP_INFO(this->get_logger(), "Desired V_x: %.2f \n",Dv_x);
        RCLCPP_INFO(this->get_logger(), "Desired V_x: %.2f \n",Dv_x);            

        RCLCPP_INFO(this->get_logger(), "Moving To the Target..");

        motion_msgs::msg::MotionCtrl cmd;
        cmd.mode_mark = false;
        cmd.value.forward = forward_speed_;
        cmd.value.left = rotation_speed_;
        cmd.value.up = 1.0;
        motion_publisher_->publish(cmd);

	nav_msgs::msg::Odometry msg;
        msg.twist.twist.linear.x = forward_speed_;
        msg.twist.twist.angular.z = rotation_speed_;

        velocity_publisher_->publish(msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr diablo_pose_subscriber_;
    rclcpp::Publisher<motion_msgs::msg::MotionCtrl>::SharedPtr motion_publisher_;
    rclcpp::Publisher<data_msgs::msg::VelocityData>::SharedPtr data_publisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr velocity_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
    bool  firstMsg;
    double initial_x;
    double initial_y;
    double Kv;
    double Kw;
    double x_c, y_c, yaw_c;
    double Dv_y, Dv_x;
    double target_x, target_y;
    vector<pair<double, double>> path_vector;
    int num_points;
    double shortest_d;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiabloSquarePathNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
