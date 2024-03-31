#include <iosfwd> // contains forward definitions for iostream objects
#include <iostream>
#include "turtlelib/se2d.hpp"

namespace turtlelib
{

std::ostream & operator<<(std::ostream & os, const Twist2D & tw)
{
  os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
  return os;
}

std::istream & operator>>(std::istream & is, Twist2D & tw)
{
  char ch;
  ch = is.peek();
  if (ch != '[') {
    is >> tw.omega;
    is >> tw.x;
    is >> tw.y;
  }
  // if the first one starts with [
  else {
    //consume "["
    is.get();
    is >> tw.omega;
    is >> tw.x;
    is >> tw.y;
  }
  return is;
}


Transform2D::Transform2D()
: linear{Vector2D{0, 0}},
  angular{0}
{}


Transform2D::Transform2D(Vector2D trans)
: linear{trans},
  angular{0}
{}


Transform2D::Transform2D(double radians)
: linear{Vector2D{0, 0}},
  angular{radians}
{}

Transform2D::Transform2D(Vector2D trans, double radians)
: linear{trans},
  angular{radians}
{}

Point2D Transform2D::operator()(Point2D p) const
{
  Point2D newPt;
  newPt.x = p.x * cos(angular) - sin(angular) * p.y + linear.x;
  newPt.y = p.x * sin(angular) + cos(angular) * p.y + linear.y;
  return newPt;
}

Vector2D Transform2D::operator()(Vector2D v) const
{
  const auto new_x = v.x * cos(angular) - sin(angular) * v.y;     //for vector there is no translation
  const auto new_y = v.x * sin(angular) + cos(angular) * v.y;
  return Vector2D{new_x, new_y};
}

Twist2D Transform2D::operator()(Twist2D v) const
{
  Twist2D newTwist;
  newTwist.x = v.omega * linear.y + v.x * cos(angular) - v.y * sin(angular);
  newTwist.y = v.omega * (-linear.x) + v.x * sin(angular) + v.y * cos(angular);
  newTwist.omega = v.omega;
  return newTwist;
}

Transform2D Transform2D::inv() const
{
  // Transform2D newTrans;
  // newTrans.angular = -angular;
  // newTrans.linear.x = -(linear.x * cos(newTrans.angular)) - linear.y * sin(newTrans.angular);
  // newTrans.linear.y = linear.y * cos(newTrans.angular) - (linear.x * sin(newTrans.angular));
  // return newTrans;
  Transform2D newTrans;
  newTrans.linear.x = -linear.x * cos(angular) - linear.y * sin(angular);
  newTrans.linear.y = -linear.y * cos(angular) + linear.x * sin(angular);
  auto oldAngular = angular;
  newTrans.angular = -oldAngular;
  return newTrans;
}

Transform2D & Transform2D::operator*=(const Transform2D & rhs)
{
  const auto new_x = rhs.linear.x * cos(angular) - rhs.linear.y * sin(angular) + linear.x;
  const auto new_y = rhs.linear.x * sin(angular) + rhs.linear.y * cos(angular) + linear.y;
  const auto newRotationAngle = normalize_angle(angular + rhs.angular);
  linear.x = new_x;
  linear.y = new_y;
  angular = newRotationAngle;

  return *this;
}

Vector2D Transform2D::translation() const
{
  return linear;
}


double Transform2D::rotation() const
{
  return angular;
}


std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
{
  os << "deg: " << rad2deg(tf.angular) << " x: " << tf.linear.x << " y: " << tf.linear.y;
  return os;
}


std::istream & operator>>(std::istream & is, Transform2D & tf)
{
  // Remove whitespace
  is >> std::ws;
  double ang_val, x_val, y_val;
  std::string label_ang, label_x, label_y;
  char c = is.peek();
  if (c == 'd') {
    is >> label_ang >> ang_val >> label_x >> x_val >> label_y >> y_val;
  } else {
    is >> ang_val >> x_val >> y_val;
  }
  // Read \n character?????????????????
  c = is.get();
  tf = Transform2D(Vector2D{x_val, y_val}, deg2rad(ang_val));

  return is;
}


Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
{
  return lhs *= rhs;
}

Transform2D integrate_twist(Twist2D tw){
  if(tw.omega == 0.0){
    return Transform2D(Vector2D{tw.x, tw.y}, 0.0);
  }
  else{
    const auto xs = tw.y/tw.omega;
    const auto ys = -tw.x/tw.omega;
    Transform2D T_sb = Transform2D(Vector2D{xs, ys});
    Transform2D T_ssprime = Transform2D(tw.omega);
    Transform2D Tbbprime = (T_sb.inv()*T_ssprime)*T_sb;
    return Tbbprime;
  }
}

  Twist2D differentiate_transform(const Transform2D & T_bB)
  {
      // Takes in T_bB and outputs Vb that results in T_bB

      // Check for pure translation
      if(almost_equal(T_bB.translation().y, 0.0) && almost_equal(T_bB.rotation(), 0.0))
      {
          return Twist2D{0, T_bB.translation().x, 0};
      }
      else
      {
          // y = R (1 - cos(theta))
          double R = fabs(T_bB.translation().y / (1 - cos(T_bB.rotation())));

          // Populate twist
          // Vb.x = R * theta_dot * cos(theta)
          return Twist2D{T_bB.rotation(), R * T_bB.rotation(), 0};
      }

      // Unanticipated error
      throw std::runtime_error("Unanticipated error in differentiate_transform");
  }

}
