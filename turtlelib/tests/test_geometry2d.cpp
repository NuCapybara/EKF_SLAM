#include "turtlelib/geometry2d.hpp"
#include <turtlelib/lidar.hpp>
#include <string>
#include <sstream>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <iostream>
using namespace std;


namespace turtlelib
{
TEST_CASE("almost_equal", "geometry2d") {
  REQUIRE(almost_equal(0.1, 0.12) == 0);
  REQUIRE(almost_equal(0.0001, 0.0001 + 1.0e-12) == 1);
  REQUIRE(almost_equal(10, 10) == 1);
}

TEST_CASE("deg2rad", "geometry2d")
{
  std::vector<double> test1 = {
    deg2rad(150),
    deg2rad(90),
    deg2rad(20),
    deg2rad(320),
    deg2rad(480)
  };
  REQUIRE_THAT(test1[0], Catch::Matchers::WithinAbs(2.61799, 1e-2));
  REQUIRE_THAT(test1[1], Catch::Matchers::WithinAbs(1.5708, 1e-2));
  REQUIRE_THAT(test1[2], Catch::Matchers::WithinAbs(0.349066, 1e-2));
  REQUIRE_THAT(test1[3], Catch::Matchers::WithinAbs(5.58505, 1e-2));
  REQUIRE_THAT(test1[4], Catch::Matchers::WithinAbs(8.37758, 1e-2));
}

TEST_CASE("rad2deg", "geometry2d") {
  std::vector<double> test2 = {
    rad2deg(PI),
    rad2deg(-PI),
    rad2deg(0),
    rad2deg(-PI / 4),
  };
  REQUIRE_THAT(test2[0], Catch::Matchers::WithinAbs(180, 1e-2));
  REQUIRE_THAT(test2[1], Catch::Matchers::WithinAbs(-180, 1e-2));
  REQUIRE_THAT(test2[2], Catch::Matchers::WithinAbs(0, 1e-2));
  REQUIRE_THAT(test2[3], Catch::Matchers::WithinAbs(-45, 1e-2));
}


TEST_CASE("normalize_angle", "geometry2d") {
  std::vector<double> test2 = {
    normalize_angle(PI),
    normalize_angle(-PI),
    normalize_angle(0),
    normalize_angle(-PI / 4),
    normalize_angle(3 * PI / 2),
    normalize_angle(-5 * PI / 2),
  };
  REQUIRE_THAT(test2[0], Catch::Matchers::WithinAbs(PI, 1e-2));
  REQUIRE_THAT(test2[1], Catch::Matchers::WithinAbs(PI, 1e-2));
  REQUIRE_THAT(test2[2], Catch::Matchers::WithinAbs(0, 1e-2));
  REQUIRE_THAT(test2[3], Catch::Matchers::WithinAbs(-PI / 4.0, 1e-2));
  REQUIRE_THAT(test2[4], Catch::Matchers::WithinAbs(-PI / 2.0, 1e-2));
  REQUIRE_THAT(test2[5], Catch::Matchers::WithinAbs(-PI / 2.0, 1e-2));
}

TEST_CASE("operator+", "point2D")
{
  Point2D p1 = Point2D{1, 2};
  Vector2D v2 = Vector2D{3, 4};
  Point2D sum = p1 + v2;
  REQUIRE_THAT(sum.x, Catch::Matchers::WithinAbs(4.0, 1e-5));
  REQUIRE_THAT(sum.y, Catch::Matchers::WithinAbs(6.0, 1e-5));
}

TEST_CASE("operator-", "point2D")
{
  Point2D p1 = Point2D{10, 11};
  Point2D p2 = Point2D{2, 5};
  Vector2D diff = p1 - p2;
  REQUIRE_THAT(diff.x, Catch::Matchers::WithinAbs(8.0, 1e-5));
  REQUIRE_THAT(diff.y, Catch::Matchers::WithinAbs(6.0, 1e-5));
}

TEST_CASE("operator<<", "Point2D)")
{
  Point2D origin;
  std::stringstream ss1;
  ss1 << origin;
  REQUIRE(ss1.str() == "[0 0]");


  Point2D pt2;
  pt2.x = 4.0;
  pt2.y = 5.0;
  std::stringstream ss2;
  ss2 << pt2;
  REQUIRE(ss2.str() == "[4 5]");

  Vector2D v2;
  std::stringstream ssv2;
  ssv2 << v2;
  REQUIRE(ssv2.str() == "[0 0]");

  Vector2D v3;
  v3.x = 1.0;
  v3.y = 5.0;
  std::stringstream ss3;
  ss3 << v3;
  REQUIRE(ss3.str() == "[1 5]");
}


TEST_CASE("operator>>", "Point2D)")
{
  Point2D pt3;
  std::istringstream input1("[3.5 -2.0]");
  input1 >> pt3;
  REQUIRE(pt3.x == 3.5);
  REQUIRE(pt3.y == -2.0);


  Point2D pt4;
  std::istringstream input2("[5 3]");
  input2 >> pt4;
  REQUIRE(pt4.x == 5.0);
  REQUIRE(pt4.y == 3.0);

  Point2D pt5;
  std::istringstream input3("2 3");
  input3 >> pt5;
  REQUIRE(pt5.x == 2.0);
  REQUIRE(pt5.y == 3.0);


  Vector2D v4;
  std::istringstream input4("[3.5 -2.0]");
  input4 >> v4;
  REQUIRE(v4.x == 3.5);
  REQUIRE(v4.y == -2.0);

  Vector2D v5;
  std::istringstream input5("5.4 -2.3");
  input5 >> v5;
  REQUIRE(v5.x == 5.4);
  REQUIRE(v5.y == -2.3);
}

TEST_CASE("operator +=", "Vector") {
  Vector2D v1{2, 5};
  Vector2D v2{11, 15};
  v1 += v2;
  REQUIRE_THAT(v1.x, Catch::Matchers::WithinAbs(13.0, 1e-5));
  REQUIRE_THAT(v1.y, Catch::Matchers::WithinAbs(20.0, 1e-5));
}

TEST_CASE("operator +", "Vector") {
  Vector2D v1{2, 5};
  Vector2D v2{11, 15};
  Vector2D v3 = v1 + v2;
  REQUIRE_THAT(v3.x, Catch::Matchers::WithinAbs(13.0, 1e-5));
  REQUIRE_THAT(v3.y, Catch::Matchers::WithinAbs(20.0, 1e-5));
}

TEST_CASE("operator -", "Vector") {
  Vector2D v1{2, 5};
  Vector2D v2{11, 15};
  Vector2D v3 = v1 - v2;
  REQUIRE_THAT(v3.x, Catch::Matchers::WithinAbs(-9.0, 1e-5));
  REQUIRE_THAT(v3.y, Catch::Matchers::WithinAbs(-10.0, 1e-5));
}

TEST_CASE("operator -=", "Vector") {
  Vector2D v1{2, 5};
  Vector2D v2{11, 15};
  v1 -= v2;
  REQUIRE_THAT(v1.x, Catch::Matchers::WithinAbs(-9.0, 1e-5));
  REQUIRE_THAT(v1.y, Catch::Matchers::WithinAbs(-10.0, 1e-5));
}

TEST_CASE("operator *", "Vector") {
  Vector2D v1{2, 5};
  double scalar = 2;
  Vector2D v3 = v1 * scalar;
  REQUIRE_THAT(v3.x, Catch::Matchers::WithinAbs(4, 1e-5));
  REQUIRE_THAT(v3.y, Catch::Matchers::WithinAbs(10.0, 1e-5));
}


TEST_CASE("operator *=", "Vector") {
  Vector2D v1{2, 5};
  double scalar = 2;
  v1 *= scalar;
  REQUIRE_THAT(v1.x, Catch::Matchers::WithinAbs(4, 1e-5));
  REQUIRE_THAT(v1.y, Catch::Matchers::WithinAbs(10.0, 1e-5));
}

TEST_CASE("dot", "Vector") {
  Vector2D v1{2, 5};
  Vector2D v2{20, 15};
  double v3 = dot(v1, v2);
  REQUIRE_THAT(v3, Catch::Matchers::WithinAbs(115, 1e-5));
}


TEST_CASE("magnitude", "Vector") {
  Vector2D v1{2, 5};
  double mag = magnitude(v1);
  REQUIRE_THAT(mag, Catch::Matchers::WithinAbs(5.3851648, 1e-5));
}


TEST_CASE("angle", "Vector") {
  Vector2D v1{0, 1};
  Vector2D v2{1, 0};
  double ang_read = angle(v1, v2);
  REQUIRE_THAT(ang_read, Catch::Matchers::WithinAbs(deg2rad(90), 1e-5));
}

TEST_CASE("if_collide", "Lidar"){
  Point2D robot{0, 0};
  Point2D column{-2, 4};
  double column_radius = 3;
  double max_range = 10;
  double theta = deg2rad(0);
  bool collide = if_intersect(column, robot, max_range, theta, column_radius);
  REQUIRE(collide == 1);
  Point2D b = intersectPoint(column, robot, max_range, theta, column_radius);
  REQUIRE_THAT(b.x, Catch::Matchers::WithinAbs(0, 1e-5));
  REQUIRE_THAT(b.y, Catch::Matchers::WithinAbs(1.7639320225, 1e-5));
  // cout << b.x << " " << b.y << endl;
}

TEST_CASE("if_collide_test2", "Lidar"){
  Point2D robot{0, 5};
  Point2D column{5, 5};
  double column_radius = 1;
  double max_range = 10;
  double theta = deg2rad(90);
  bool collide = if_intersect(column, robot, max_range, theta, column_radius);
  REQUIRE(collide == 1);
  Point2D b = intersectPoint(column, robot, max_range, theta, column_radius);
  // cout << a.x << " " << a.y << endl;
  cout << b.x << " " << b.y << endl;
  // REQUIRE_THAT(a.x, Catch::Matchers::WithinAbs(6, 1e-5));
  // REQUIRE_THAT(a.y, Catch::Matchers::WithinAbs(5, 1e-5));

  REQUIRE_THAT(b.x, Catch::Matchers::WithinAbs(4, 1e-5));
  REQUIRE_THAT(b.y, Catch::Matchers::WithinAbs(5, 1e-5));

}



TEST_CASE("if_collide_test3", "Lidar"){
  Point2D robot{0, 0};
  Point2D column{5, 5};
  double column_radius = 1;
  double max_range = 10;
  double theta = deg2rad(45);
  bool collide = if_intersect(column, robot, max_range, theta, column_radius);
  REQUIRE(collide == 1);
  // Point2D a = intersectPointA(column, robot, max_range, theta, column_radius);
  Point2D b = intersectPoint(column, robot, max_range, theta, column_radius);
  // cout << a.x << " " << a.y << endl;
  cout << b.x << " " << b.y << endl;
  // REQUIRE_THAT(a.x, Catch::Matchers::WithinAbs(5.707, 1e-2));
  // REQUIRE_THAT(a.y, Catch::Matchers::WithinAbs(5.707, 1e-2));

  REQUIRE_THAT(b.x, Catch::Matchers::WithinAbs(4.292, 1e-2));
  REQUIRE_THAT(b.y, Catch::Matchers::WithinAbs(4.292, 1e-2));

}

TEST_CASE("if_collide_test4", "Lidar"){
  Point2D robot{0, 0};
  Point2D column{0, 5};
  double column_radius = 4;
  double max_range = 10;
  double theta = deg2rad(30);
  bool collide = if_intersect(column, robot, max_range, theta, column_radius);
  REQUIRE(collide == 1);
  // Point2D a = intersectPointA(column, robot, max_range, theta, column_radius);
  Point2D b = intersectPoint(column, robot, max_range, theta, column_radius);

  REQUIRE_THAT(b.x, Catch::Matchers::WithinAbs(0.60381, 1e-2));
  REQUIRE_THAT(b.y, Catch::Matchers::WithinAbs(1.04583, 1e-2));

}

TEST_CASE("if_collide_test5", "Lidar"){
  Point2D robot{2, 0};
  Point2D column{0, 5};
  double column_radius = 4;
  double max_range = 10;
  double theta =  deg2rad(-60);
  bool collide = if_intersect(column, robot, max_range, theta, column_radius);
  REQUIRE(collide == 1);
  // Point2D a = intersectPointA(column, robot, max_range, theta, column_radius);
  Point2D b = intersectPoint(column, robot, max_range, theta, column_radius);

  REQUIRE_THAT(b.x, Catch::Matchers::WithinAbs(0.254, 1e-2));
  REQUIRE_THAT(b.y, Catch::Matchers::WithinAbs(1.0081, 1e-2));

}



}
