#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <stdexcept>

namespace turtlelib
{   DiffDrive::DiffDrive():
        w{WheelState{0.0, 0.0}},
        q{Transform2D()},
        wheel_radius{0.0},
        track_width{0.0}
    {}

    DiffDrive::DiffDrive(double wheel_radius, double track_width):
        w{WheelState{0.0, 0.0}},
        q{Transform2D()},
        wheel_radius{wheel_radius},
        track_width{track_width}
    {}

    DiffDrive::DiffDrive(Transform2D config, double wheel_radius, double track_width):
        w{WheelState{0.0, 0.0}},
        q{config},
        wheel_radius{wheel_radius},
        track_width{track_width}
    {}


    double DiffDrive::get_radius(){
        return wheel_radius;
    }

    double DiffDrive::get_track_width(){
        return track_width;
    }

    WheelState DiffDrive::get_wheel_state(){
        return w;
    }
    
    Transform2D DiffDrive::get_current_config(){
        return q;
    }
    void DiffDrive::forwardKinematics(WheelState newWheelState){
        //updating the wheel state
        double newLeft = newWheelState.l;
        double newRight = newWheelState.r;

        w.l += newLeft;
        w.r += newRight;

        //eq 13.34
        auto twist_w = (wheel_radius / track_width) * (newWheelState.r - newWheelState.l);
        auto twist_x = (wheel_radius / 2.0) * (newWheelState.l + newWheelState.r);
        auto twist_y = 0.0;

        Transform2D Tbbprime = integrate_twist(Twist2D{twist_w, twist_x, twist_y});
        // Tbbprime = Transform2D{Vector2D{Tbbprime.translation().x/2000, Tbbprime.translation().y/2000}, Tbbprime.rotation()};
        Transform2D Tsb = q;
        Transform2D Tsbprime = Tsb*Tbbprime; //curr q is in Tsb

        q = Tsbprime;
    }

    WheelState DiffDrive::inverseKinematics(Twist2D twist){
        if(almost_equal(twist.y, 0.0)){
            const auto omega = twist.omega;
            const auto tx = twist.x;
            ///MR Formula 13.34
            // auto newleftWheel = 1/(2*wheel_radius)*(2*tx - track_width*omega);
            // auto newrightWheel = 1/(2*wheel_radius)*(track_width*omega + 2*tx);
            auto d = track_width/2.0;
            auto newleftWheel = (tx-d*omega)/wheel_radius;
            auto newrightWheel = (tx+d*omega)/wheel_radius;

            return WheelState{newleftWheel, newrightWheel};
        }
        else{
            throw std::logic_error("Slip existing, twist y component not equal to 0.");
        }
    }

    void DiffDrive::set_radius(double radius){
        wheel_radius = radius;
    }

    void DiffDrive::set_track_width(double width){
        track_width = width;
    }    

}
