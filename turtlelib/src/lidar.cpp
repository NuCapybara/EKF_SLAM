#include <iosfwd> // contains forward definitions for iostream objects
#include <iostream>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include <cmath>
#include <string>



namespace turtlelib{

    bool if_intersect(Point2D column, Point2D robot, double max_range, double theta, double column_radius){
        //====================== laser vector should be robot.x+xxx, robot.y+yyy ======================
        Vector2D laser{sin(theta)*max_range, cos(theta)*max_range}; ///theta here for radians, a laser vetor in robot frame
        Vector2D U{column.x-robot.x, column.y-robot.y}; ///U is the vector from robot to column

        ///project U onto laser
        double proj_mag = (dot(U, laser)/magnitude(laser));
        Vector2D unit_vector_laser = {laser.x/magnitude(laser), laser.y/magnitude(laser)};
        Vector2D U1 = unit_vector_laser*proj_mag;  ///U1 is the projection of U onto laser

        ///VeCtor from projection vector to column center
        Vector2D U2 = U - U1; 
        double d = magnitude(U2); ///distance from robot to column center

        return (d <= column_radius);
    }


    Point2D intersectPoint(Point2D column, Point2D robot, double max_range, double theta, double column_radius){
        Vector2D laser{sin(theta)*max_range, cos(theta)*max_range}; ///theta here for radians, a laser vetor in robot frame
        std::cout << "laser: " << laser.x << " " << laser.y << std::endl;
        Vector2D U{column.x-robot.x, column.y-robot.y}; ///U is the vector from robot to column
        std::cout << "U: " << U.x << " " << U.y << std::endl;

        ///project U onto laser
        double proj_mag = (dot(U, laser)/magnitude(laser));
        ///added new lines
        if(proj_mag < 0){
            return Point2D{10000, 10000};
        }
        std::cout << "proj_mag: " << proj_mag << std::endl;
        Vector2D unit_vector_laser = {laser.x/magnitude(laser), laser.y/magnitude(laser)};
        std::cout << "unit_vector_laser: " << unit_vector_laser.x << " " << unit_vector_laser.y << std::endl;

        Vector2D U1 = unit_vector_laser*proj_mag;  ///U1 is the projection of U onto laser
        std::cout << "U1: " << U1.x << " " << U1.y << std::endl;

        ///VeCtor from projection vector to column center
        Vector2D U2 = U - U1; 
        std::cout << "U2: " << U2.x << " " << U2.y << std::endl;
        double d = magnitude(U2); ///distance from robot to column center   
        std::cout << "d: " << d << std::endl;

        if(d <= column_radius){
            double m = sqrt(pow(column_radius, 2) - pow(d, 2));
            Point2D intersectPointA = robot + U1 + unit_vector_laser*m;
            std::cout << "intersectPointA: " << intersectPointA.x << " " << intersectPointA.y << std::endl;
            ///calculate IntersectPointB
            Point2D buffer1 = robot + U1;
            std::cout << "buffer1 " << buffer1.x << " " << buffer1.y << std::endl;
            Vector2D buffer2 = unit_vector_laser*m*(-1);
            std::cout << "buffer2 " << buffer2.x << " " << buffer2.y << std::endl;
            Point2D intersectPointB = buffer1 + buffer2;
            std::cout << "intersectPointB: " << intersectPointB.x << " " << intersectPointB.y << std::endl;

            double RobotToA = magnitude(intersectPointA - robot);
            double RobotToB = magnitude(intersectPointB - robot);
            if(RobotToA <= RobotToB){
                return intersectPointA;
            }
            else{
                return intersectPointB;
            }
        }
        else{
            return Point2D{10000, 10000};
        }
    }


}