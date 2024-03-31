#include "turtlelib/geometry2d.hpp"
#include <iostream>
#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"
#include <cmath>

int main()
{
  turtlelib::Transform2D Tab = {};
  turtlelib::Transform2D Tbc = {};

  std::cout << "Enter transform T_{a,b}:" << std::endl;
  std::cin >> Tab;
  std::cout << "Enter transform T_{b,c}:" << std::endl;
  std::cin >> Tbc;

  turtlelib::Transform2D Tba = Tab.inv();
  turtlelib::Transform2D Tcb = Tbc.inv();
  turtlelib::Transform2D Tac = Tab * Tbc;
  turtlelib::Transform2D Tca = Tac.inv();

  std::cout << "T_{a,b}:" << Tab << std::endl;
  std::cout << "T_{b,a}:" << Tba << std::endl;
  std::cout << "T_{b,c}:" << Tbc << std::endl;
  std::cout << "T_{c,b}:" << Tcb << std::endl;
  std::cout << "T_{a,c}:" << Tac << std::endl;
  std::cout << "T_{c,a}:" << Tca << std::endl;

  turtlelib::Point2D pt;
  std::cout << "Enter point p_a:" << std::endl;
  std::cin >> pt;
  std::cout << "p_a: " << pt << std::endl;
  std::cout << "p_b: " << Tba(pt) << std::endl;
  std::cout << "p_c: " << Tca(pt) << std::endl;


  turtlelib::Vector2D vec;
  std::cout << "Enter vector v_b:" << std::endl;
  std::cin >> vec;

  double mag = sqrt(vec.x * vec.x + vec.y * vec.y);
  double normalx = vec.x / mag;
  double normaly = vec.y / mag;
  std::cout << "v_bhat: " << "[" << normalx << " " << normaly << "]" << std::endl;
  std::cout << "v_a: " << Tab(vec) << std::endl;
  std::cout << "v_b: " << vec << std::endl;
  std::cout << "v_c: " << Tcb(vec) << std::endl;


  turtlelib::Twist2D V_b;
  std::cout << "Enter twist V_b:" << std::endl;
  std::cin >> V_b;
  // print out different transforms
  std::cout << "V_a:" << Tab(V_b) << std::endl;
  std::cout << "V_b:" << V_b << std::endl;
  std::cout << "V_c:" << Tcb(V_b) << std::endl;
  return 0;

}
