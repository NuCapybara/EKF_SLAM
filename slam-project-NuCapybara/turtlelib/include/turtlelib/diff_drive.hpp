#ifndef TURTLELIB_DIFFDRIVE_HPP_INCLUDE_GUARD
#define TURTLELIB_DIFFDRIVE_HPP_INCLUDE_GUARD
/// \file
/// \brief Two-dimensional geometric primitives.


#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"


namespace turtlelib
{   /// \brief struct to store wheel state
    struct WheelState{
        /// @brief left wheel state, in radians
        double l = 0.0;
        /// @brief right wheel state in radians
        double r = 0.0;
    };

    /// @brief class to model the kinematics of a differential drive robot
    class DiffDrive{
    public:
        /// @brief wheel status on right and left side
        WheelState w;

        /// @brief current configuration of robot in real world
        Transform2D q;
        /// @brief wheel radius
        double wheel_radius;
        /// @brief width between two wheels
        double track_width;
    
    public:
        /// @brief models the kinematics of a differential drive robot with default wheel track and wheel radius.
        DiffDrive();

        /// @brief models the kinematics of a differential drive robot with a given wheel track and wheel radius.
        /// @param wheel_radius
        /// @param track_width 
        DiffDrive(double wheel_radius, double track_width);

        /// @brief models the kinematics of a differential drive robot with a given wheel track and wheel radius.
        /// @param config 
        /// @param wheel_radius 
        /// @param track_width 
        DiffDrive(Transform2D config, double wheel_radius, double track_width);

        /// @brief get radius of wheel
        /// @return double
        double get_radius();

        /// @brief get track width
        /// @return double
        double get_track_width();
        
        /// @brief get current wheel state
        /// @return WheelState 
        WheelState get_wheel_state();

        /// @brief get current configuration q of robot in real world
        /// @return current configuration Transform2D
        Transform2D get_current_config();

        /// @brief  Given new wheel positions, update the configuration
        /// @param newWheelState 
        void forwardKinematics(WheelState newWheelState);

        /// @brief Compute the wheel velocities required to make the robot move at a given body twist.
        /// @param twist 
        /// @return wheel velocity
        WheelState inverseKinematics(Twist2D twist);

        /// @brief set radius of wheel
        /// @param radius
        void set_radius(double radius);

        /// @brief set track width
        /// @param width
        void set_track_width(double width);
        
    }; 
}

#endif