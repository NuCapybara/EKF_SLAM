#include "turtlelib/geometry2d.hpp"
#include <iostream>
#include <cmath>

namespace turtlelib
{

std::ostream & operator<<(std::ostream & os, const Point2D & p)
{
  os << "[" << p.x << " " << p.y << "]";
  return os;
}

std::istream & operator>>(std::istream & is, Point2D & p)
{
  char ch;
  ch = is.peek();
  if (ch != '[') {
    is >> p.x;
    is >> p.y;
  }
  // if the first one starts with [
  else {
    //consume "["
    is.get();
    //the next one for input is p.x
    is >> p.x;
    // is >> trash;
    is >> p.y;
  }

  return is;
}

double normalize_angle(double rad)
{
  while (rad <= -PI) {
    rad += 2 * PI;
  }
  while (rad > PI) {
    rad -= 2 * PI;
  }
  return rad;
}


Vector2D operator-(const Point2D & head, const Point2D & tail)
{
  Vector2D v;
  v.x = head.x - tail.x;
  v.y = head.y - tail.y;
  return v;
}

Vector2D & Vector2D::operator+=(const Vector2D& rhs){
  x += rhs.x;
  y += rhs.y;
  return *this;
}

Vector2D operator+(const Vector2D& lhs, const Vector2D& rhs)
{
  Vector2D result = lhs;
  result += rhs;
  return result;
}


Vector2D & Vector2D::operator-=(const Vector2D & rhs){
  x -= rhs.x;
  y -= rhs.y;
  return *this;
}

Vector2D operator-(const Vector2D& lhs, const Vector2D& rhs){
  Vector2D result = lhs;
  result -= rhs;
  return result;
}

Vector2D & Vector2D::operator*=(const double & scalar){
  x *= scalar;
  y *= scalar;
  return *this;
}

Vector2D operator*(Vector2D vec, const double & scalar){
  return vec*=scalar;
}


double dot(Vector2D v1, Vector2D v2){
  return v1.x*v2.x + v1.y*v2.y;
}

/// @brief computer the magnitude of the vector
/// @param v 
/// @return double
double magnitude(Vector2D v){
  double mag = sqrt(v.x*v.x+v.y*v.y);
  return mag;
}

/// @brief compute the angle between two vectors
/// @param v1 
/// @param v2 
/// @return angle between v1 and v2
double angle(Vector2D v1, Vector2D v2){
  double angle_read = acos(dot(v1, v2)/(magnitude(v1)*magnitude(v2)));
  return angle_read;
}

Point2D operator+(const Point2D & tail, const Vector2D & disp)
{
  Point2D pt;
  pt.x = tail.x + disp.x;
  pt.y = tail.y + disp.y;
  return pt;
}

std::ostream & operator<<(std::ostream & os, const Vector2D & v)
{
  os << "[" << v.x << " " << v.y << "]";
  return os;
}


std::istream & operator>>(std::istream & is, Vector2D & v)
{
  is >> std::ws;
  char input;
  input = is.peek();
  //Sceanario: the first char is [
  if (input == '[') {
    is.get();         //consume [
    double x;
    double y;
    is >> x;
    is.get();         //consumes the whitespace between x and y
    is >> y;
    if (is.get() == ']') {
      v.x = x;
      v.y = y;
    } else {
      //if the last one is not ], then fails
      is.setstate(std::ios::failbit);
    }
  } else {
    double x;
    double y;
    is >> x;
    is >> y;
    v.x = x;
    v.y = y;
  }
  return is;
}

}
