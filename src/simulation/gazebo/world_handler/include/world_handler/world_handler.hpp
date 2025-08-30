#pragma once

#include <rclcpp/rclcpp.hpp>
#include <signal.h>

namespace gazebo
{
    class WorldHandler : public rclcpp::Node
    {
        public:
            WorldHandler();
            ~WorldHandler();
        private:
            rclcpp::TimerBase::SharedPtr timer_;
            static void sigintHandler(int sig);
            void timerCallback();
    };
}