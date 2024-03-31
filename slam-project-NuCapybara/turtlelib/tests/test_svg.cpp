#include <iosfwd> // contains forward definitions for iostream objects
#include <iostream>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <string>
#include <sstream>
#include "turtlelib/svg.hpp"
#include <fstream>


namespace turtlelib
{
TEST_CASE("svgfile", "test") {
  std::ostringstream os;
  double width = 8.50000;
  double height = 11.00000;
  double cx = 4.0;
  double cy = 3.0;

  Point2D pt;
  pt.x = cx;
  pt.y = cy;


  double x1 = 3.5;
  double x2 = 2.0;
  double y1 = 3.5;
  double y2 = 2.0;
  Vector2D v1;
  v1.x = x1;
  v1.y = y1;
  Vector2D v2;
  v2.x = x2;
  v2.y = y2;
  Transform2D t(Vector2D{0, 0}, 0);
  Transform2D t2(Vector2D{1, 0}, 0);


  std::string stroke = "purple";
  std::string text = "a";
  OpenTag(os, width, height);
  defTag(os);
  drawCircle(os, pt, t, stroke);
  drawVector(os, v1, v2, t, stroke);
  groupTag(os, t2, text);
  endSvg(os);

  std::string result = os.str();
  std::ofstream("output.svg") << os.str();
}
}
