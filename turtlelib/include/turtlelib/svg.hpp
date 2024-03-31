#ifndef TURTLELIB_SVG_INCLUDE_GUARD_HPP
#define TURTLELIB_SVG_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.
#include<iosfwd> // contains forward definitions for iostream objects
#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"
#include <string>

namespace turtlelib
{
    /// \brief generate the svg tag for view box
    /// \param width - the width of viewbox
    /// \param os - the os stream
    /// \param height - the height of viewbox
    // \return nothing
    void OpenTag(std::ostream & os, double width, double height);

    /// \brief create the def  tag
    /// \param os - the os stream
    /// \return nothing
    void defTag(std::ostream & os);

    /// \brief draw the circle
    /// \param os - the os stream
    /// \param pt - the point of center of circle
    /// \param t -the transformation you wanna apply on the circle
    /// \param stroke -the color of the stroke
    /// \return nothing
    void drawCircle(std::ostream & os, Point2D pt, Transform2D t, std::string stroke);

    /// \brief draw the vector
    /// \param os - the os stream
    /// \param v1 - the vector head
    /// \param v2 - the vector tail
    /// \param t -the transformation you wanna apply on the vector
    /// \param stroke -the color of the stroke
    /// \return nothing
    void drawVector(std::ostream & os, Vector2D v1, Vector2D v2, Transform2D t, std::string stroke);


    /// \brief draw the vector
    /// \param os - the os stream
    /// \param t -the transformation you wanna apply on the vector
    /// \param text -the text you wanna write
    /// \return nothing
    void groupTag(std::ostream & os, Transform2D t, std::string text);

    /// \brief create the svg closing tag
    /// \param os - the os stream
    /// \return nothing
    void endSvg(std::ostream & os);
}

#endif