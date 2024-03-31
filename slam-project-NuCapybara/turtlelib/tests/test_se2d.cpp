#include <iosfwd> // contains forward definitions for iostream objects
#include <iostream>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <string>
#include <sstream>
#include "turtlelib/geometry2d.hpp"


namespace turtlelib
{
double epsilon = 0.01;
TEST_CASE("Empty Constructor", "[transform]")
{
  Transform2D Tran;
  REQUIRE(Tran.translation().x == 0.0);
  REQUIRE(Tran.translation().y == 0.0);
  REQUIRE(Tran.rotation() == 0.0);
}


TEST_CASE("Translation Only Constructor", "[transform]")
{
  double my_x = 5;
  double my_y = 6;
  Transform2D T(Vector2D{my_x, my_y});
  REQUIRE(T.rotation() == 0.0);
  REQUIRE(T.translation().x == my_x);
  REQUIRE(T.translation().y == my_y);
}

TEST_CASE("Rotation Only Constructor", "[transform]")
{
  double ang = PI / 4;
  Transform2D T(ang);
  REQUIRE(T.rotation() == ang);
  REQUIRE(T.translation().x == 0.0);
  REQUIRE(T.translation().y == 0.0);
}

TEST_CASE("Rotation and Translation", "[transform]")
{
  double x = 3.0;
  double y = 5.0;
  double ang = 6.2876;
  Transform2D T(Vector2D{x, y}, ang);
  REQUIRE(T.rotation() == ang);
  REQUIRE(T.translation().x == x);
  REQUIRE(T.translation().y == y);
}

TEST_CASE("operator()(Vector2D v)", "[transform]")
{
  double test_rot = PI / 2.0;
  double test_x = 0.0;
  double test_y = 1.0;
  Transform2D T_ab{{test_x, test_y}, test_rot};
  Vector2D v_b{1, 1};
  Vector2D v_a = T_ab(v_b);
  REQUIRE_THAT(v_a.x, Catch::Matchers::WithinAbs(-1, 1e-5));
  REQUIRE_THAT(v_a.y, Catch::Matchers::WithinAbs(1, 1e-5));
}

TEST_CASE("Transforming a twist", "[se2d]") {     //Kyle Wang
  Transform2D tf = {Vector2D{0.2, 1.1}, 0.34};
  Twist2D tw = {1.0, 0.707, 0.707};
  Twist2D ans = tf(tw);
  REQUIRE_THAT(ans.omega, Catch::Matchers::WithinRel(1.0));
  REQUIRE_THAT(ans.x, Catch::Matchers::WithinRel(1.531, 0.001));
  REQUIRE_THAT(ans.y, Catch::Matchers::WithinRel(0.702, 0.001));
}

TEST_CASE("Twist2D operator<<", "[twist2d]")     // Carter, DiOrio
{
  turtlelib::Twist2D tw{1.1, 2.0, 3.0};
  std::ostringstream oss;
  oss << tw;
  REQUIRE(oss.str() == "[1.1 2 3]");
}


TEST_CASE("Twist2D operator>>", "[twist2d]")     //Carter, DiOrio
{
  std::istringstream iss("[1.0 2.0 3.0]");
  turtlelib::Twist2D tw;
  iss >> tw;
  REQUIRE_THAT(tw.omega, Catch::Matchers::WithinAbs(1.0, epsilon));
  REQUIRE_THAT(tw.x, Catch::Matchers::WithinAbs(2.0, epsilon));
  REQUIRE_THAT(tw.y, Catch::Matchers::WithinAbs(3.0, epsilon));
}


TEST_CASE("Transform2D operator<<", "[transform2d]")     // Carter, DiOrio
{
  turtlelib::Transform2D tf{Vector2D{3.0, 5.0}, turtlelib::deg2rad(90.0)};
  std::ostringstream oss;
  oss << tf;
  REQUIRE(oss.str() == "deg: 90 x: 3 y: 5");
}


TEST_CASE("Transform2D operator>>", "[transform2d]")     // Carter, DiOrio
{
  std::istringstream iss("90.0 3.0 5.0");
  turtlelib::Transform2D tf;
  iss >> tf;
  REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(3.0, epsilon));
  REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(5.0, epsilon));
  REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(turtlelib::deg2rad(90.0), epsilon));
}

TEST_CASE("Multiply operator *", "[transform2D]") {    // Kyle Wang
  Transform2D tf1 = {Vector2D{1.2, -2.2}, 0.6};
  Transform2D tf2 = {Vector2D{0.3, 4.1}, -0.1};
  Transform2D tf3 = tf1 * tf2;
  REQUIRE_THAT(tf3.rotation(), Catch::Matchers::WithinRel(0.5));
  REQUIRE_THAT(tf3.translation().x, Catch::Matchers::WithinRel(-0.867, 0.001));
  REQUIRE_THAT(tf3.translation().y, Catch::Matchers::WithinRel(1.353, 0.001));
}


TEST_CASE("Inverting a transform", "[transform2D]") {    //Kyle Wang
  Transform2D tf = {Vector2D{-1.2, 0.35}, 0.29};
  Transform2D tfInv = tf.inv();
  REQUIRE_THAT(tfInv.rotation(), Catch::Matchers::WithinRel(-0.29));
  REQUIRE_THAT(tfInv.translation().x, Catch::Matchers::WithinRel(1.050, 0.001));
  REQUIRE_THAT(tfInv.translation().y, Catch::Matchers::WithinRel(-0.679, 0.001));
}

  TEST_CASE("integrate_twist", "Twist2D")
  {
      // Only Translation
      Twist2D tw = Twist2D{0, 1, 2};
      Transform2D Tbbprime = integrate_twist(tw);
      REQUIRE_THAT(Tbbprime.translation().x, Catch::Matchers::WithinAbs(1.0, 1e-5));
      REQUIRE_THAT(Tbbprime.translation().y, Catch::Matchers::WithinAbs(2.0, 1e-5));
      REQUIRE_THAT(Tbbprime.rotation(), Catch::Matchers::WithinAbs(0.0, 1e-5));
      // Only Rotation
      tw = Twist2D{PI, 0, 0};
      Tbbprime = integrate_twist(tw);
      REQUIRE_THAT(Tbbprime.translation().x, Catch::Matchers::WithinAbs(0.0, 1e-5));
      REQUIRE_THAT(Tbbprime.translation().y, Catch::Matchers::WithinAbs(0.0, 1e-5));
      REQUIRE_THAT(Tbbprime.rotation(), Catch::Matchers::WithinAbs(PI, 1e-5));
      // Translation + Rotation
      tw = Twist2D{-1.24, -2.15,-2.92};
      Tbbprime = integrate_twist(tw);
      REQUIRE_THAT(Tbbprime.translation().x, Catch::Matchers::WithinAbs(-3.229863264722, 1e-5));
      REQUIRE_THAT(Tbbprime.translation().y, Catch::Matchers::WithinAbs(-1.05645265317421, 1e-5));
      REQUIRE_THAT(Tbbprime.rotation(), Catch::Matchers::WithinAbs(-1.24, 1e-5));
  }


  TEST_CASE("Drive forward", "DiffDrive"){
        double track = 1.0;
        double rad = 1.0;
        DiffDrive dd(track, rad);
        REQUIRE(dd.get_current_config().translation().x  == 0);
        REQUIRE(dd.get_current_config().translation().y  == 0);
        REQUIRE(dd.get_current_config().rotation() == 0);
        REQUIRE(dd.get_wheel_state().l == 0);
        REQUIRE(dd.get_wheel_state().r == 0);
        // define desired twist: move straight one unit in x direction
        Twist2D tw = Twist2D{0,1.0, 0.0};
        WheelState ws = dd.inverseKinematics(tw);
        REQUIRE_THAT(ws.l, Catch::Matchers::WithinAbs(rad, 1e-5));
        REQUIRE_THAT(ws.r, Catch::Matchers::WithinAbs(rad, 1e-5));
        dd.forwardKinematics(WheelState{ws.l,ws.r});
        REQUIRE_THAT(dd.get_current_config().translation().x, Catch::Matchers::WithinAbs(1.0, 1e-5));
        REQUIRE_THAT(dd.get_current_config().translation().y, Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(dd.get_current_config().rotation(), Catch::Matchers::WithinAbs(0.0, 1e-5));
        // REQUIRE(1 == 0);
    }

    TEST_CASE("Rotate Only", "DiffDrive"){
        double track = 1.0;
        double rad = 1.0;
        DiffDrive dd(track, rad);
        REQUIRE(dd.get_current_config().translation().x == 0);
        REQUIRE(dd.get_current_config().translation().y == 0);
        REQUIRE(dd.get_current_config().rotation() == 0);
        REQUIRE(dd.get_wheel_state().l == 0);
        REQUIRE(dd.get_wheel_state().r == 0);
        // define desired twist: Rotate to PI
        Twist2D tw = Twist2D{PI, 0.0, 0.0};
        WheelState ws = dd.inverseKinematics(tw);
        REQUIRE_THAT(ws.l, Catch::Matchers::WithinAbs(-0.5*rad*PI, 1e-5));
        REQUIRE_THAT(ws.r, Catch::Matchers::WithinAbs(0.5*rad*PI, 1e-5));
        dd.forwardKinematics(ws);
        REQUIRE_THAT(dd.get_current_config().translation().x , Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(dd.get_current_config().translation().y, Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(dd.get_current_config().rotation(), Catch::Matchers::WithinAbs(PI, 1e-5));
    }

    TEST_CASE("Rotate and Translate", "DiffDrive"){
        double track = 1.0;
        double rad = 1.0;
        DiffDrive dd(track, rad);
        REQUIRE(dd.get_current_config().translation().x == 0);
        REQUIRE(dd.get_current_config().translation().y == 0);
        REQUIRE(dd.get_current_config().rotation() == 0);
        REQUIRE(dd.get_wheel_state().l == 0);
        REQUIRE(dd.get_wheel_state().r == 0);
        // define desired twist: Rotate to PI and move forward 1 unit
        Twist2D tw = Twist2D{PI, 1.0, 0.0};
        WheelState ws = dd.inverseKinematics(tw);
        REQUIRE_THAT(ws.l, Catch::Matchers::WithinAbs(1-0.5*PI, 1e-5));
        REQUIRE_THAT(ws.r, Catch::Matchers::WithinAbs(1+0.5*PI, 1e-5));
        dd.forwardKinematics(WheelState{ws.l,ws.r});
        REQUIRE_THAT(dd.get_current_config().translation().x, Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(dd.get_current_config().translation().y, Catch::Matchers::WithinAbs(0.6366197724, 1e-5));
        REQUIRE_THAT(dd.get_current_config().rotation(), Catch::Matchers::WithinAbs(PI, 1e-5));
    }

    TEST_CASE("Test Exeption", "DiffDrive"){
        double track = 1.0;
        double rad = 1.0;
        DiffDrive dd(track, rad);
        // Give it an impossible twist
        Twist2D tw = Twist2D{PI,0.0, 1.0};
        CHECK_THROWS(dd.inverseKinematics(tw));
    }

    TEST_CASE("Test a Few Transforms in a Row", "DiffDrive"){
        double track = 1.0;
        double rad = 1.0;
        DiffDrive dd(track, rad);
        REQUIRE(dd.get_current_config().translation().x == 0);
        REQUIRE(dd.get_current_config().translation().y == 0);
        REQUIRE(dd.get_current_config().rotation() == 0);
        REQUIRE(dd.get_wheel_state().l == 0);
        REQUIRE(dd.get_wheel_state().r == 0);
        // define desired twist: first move 1 unit forward in x direction
        Twist2D tw = Twist2D{0, 1.0, 0.0};
        WheelState ws = dd.inverseKinematics(tw);
        dd.forwardKinematics(WheelState{ws.l,ws.r});
        REQUIRE_THAT(dd.get_current_config().translation().x, Catch::Matchers::WithinAbs(1.0, 1e-5));
        REQUIRE_THAT(dd.get_current_config().translation().y, Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(dd.get_current_config().rotation(), Catch::Matchers::WithinAbs(0.0, 1e-5));

        // then rotate pi/2
        tw = Twist2D{0.5*PI, 0.0, 0.0};
        ws = dd.inverseKinematics(tw);
        dd.forwardKinematics(WheelState{ws.l,ws.r});
        REQUIRE_THAT(dd.get_current_config().translation().x, Catch::Matchers::WithinAbs(1.0, 1e-5));
        REQUIRE_THAT(dd.get_current_config().translation().y, Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(dd.get_current_config().rotation(), Catch::Matchers::WithinAbs(0.5*PI, 1e-5));

        // Drive forward one again. should go to y = 1
        tw = Twist2D{0.0, 1.0, 0.0};
        ws = dd.inverseKinematics(tw);
        dd.forwardKinematics(ws);
        REQUIRE_THAT(dd.get_current_config().translation().x, Catch::Matchers::WithinAbs(1.0, 1e-5));
        REQUIRE_THAT(dd.get_current_config().translation().y, Catch::Matchers::WithinAbs(1.0, 1e-5));
        REQUIRE_THAT(dd.get_current_config().rotation(), Catch::Matchers::WithinAbs(0.5*PI, 1e-5));
    }



      TEST_CASE("Robot drives frowards"){                              // Shedd, Kassidy (Mine)

      DiffDrive robot{Transform2D{Vector2D{0.0, 0.0}, 0.0}, 0.033, 0.16};
      turtlelib::WheelState pos = {0.5, 0.5};

      robot.forwardKinematics(pos);
      Transform2D rs = robot.get_current_config();

      REQUIRE_THAT(rs.translation().x, Catch::Matchers::WithinRel(0.0165, 0.1e-5));
      REQUIRE_THAT(rs.translation().y, Catch::Matchers::WithinRel(0.0, 0.1e-5));
      REQUIRE_THAT(rs.rotation(), Catch::Matchers::WithinRel(0.0, 0.1e-5));   
  }
        
      TEST_CASE("Robot executes pure rotation"){                       // Shedd, Kassidy (Mine)
 
      DiffDrive robot{Transform2D{Vector2D{0.0, 0.0}, 0.0}, 0.033, 0.16};;
      turtlelib::WheelState pos = {-0.5, 0.5};

      robot.forwardKinematics(pos);
      Transform2D rs = robot.get_current_config();

      REQUIRE_THAT(rs.translation().x, Catch::Matchers::WithinRel(0.0, 1e-5));
      REQUIRE_THAT(rs.translation().y, Catch::Matchers::WithinRel(0.0, 1e-5));
      REQUIRE_THAT(rs.rotation(), Catch::Matchers::WithinRel(0.20625, 1e-4));   
  }
        
  TEST_CASE("Robot follows arc of a circle"){                     // Shedd, Kassidy (Mine)
      DiffDrive robot{Transform2D{Vector2D{0.0, 0.0}, 0.0}, 0.033, 0.16};;
      turtlelib::WheelState pos ={0.5, 2.0};

      robot.forwardKinematics(pos);
      Transform2D rs = robot.get_current_config();

      REQUIRE_THAT(rs.translation().x, Catch::Matchers::WithinRel(0.040595, 1e-4));
      REQUIRE_THAT(rs.translation().y, Catch::Matchers::WithinRel(0.0063301, 1e-5));
      REQUIRE_THAT(rs.rotation(), Catch::Matchers::WithinRel(0.30938, 1e-4));  
  }

        

}
