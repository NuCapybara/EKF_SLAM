#include<iosfwd> // contains forward definitions for iostream objects
#include <iostream>
#include"turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/svg.hpp"
#include <ostream>
#include <cmath>
#include <string>


namespace turtlelib
{
    void OpenTag(std::ostream & os, double width, double height){
        double xpixel = width*96 + 8.5/2.0*96.0;
        double ypixel = -height*96 + 11.0/2.0* 96.0;
        os << "<svg width=\"" << width << "in\" height=\"" << height << "in\" "
            << "viewBox=\"0 0 " << xpixel << " " << ypixel << "\"" << " xmlns=\"http://www.w3.org/2000/svg\">" << "\n";
    }

    void defTag(std::ostream &os) {
        os << "<defs>\n"
        << "  <marker\n"
        << "     style=\"overflow:visible\"\n"
        << "     id=\"Arrow1Sstart\"\n"
        << "     refX=\"0.0\"\n"
        << "     refY=\"0.0\"\n"
        << "     orient=\"auto\">\n"
        << "    <path\n"
        << "       transform=\"scale(0.2) translate(6,0)\"\n"
        << "       style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\"\n"
        << "       d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"\n"
        << "       />\n"
        << "  </marker>\n"
        << "</defs>\n";
    }

    void drawCircle(std::ostream & os, Point2D pt, Transform2D t, std::string stroke){
        Point2D p = t(pt);
        double cx = p.x * 96.0 + 8.5/2.0* 96.0;
        double cy = -p.y *96.0 + 11.0/2.0* 96.0;
        
        os << "<circle cx=\""<< cx << "\" cy=\""<< cy <<"\" r=\" "<< "3.0" <<"\" stroke=\"" << stroke << "\" fill=\"" << stroke <<"\" stroke-width=\"1\" />\n";
    }

    void drawVector(std::ostream & os, Vector2D v1, Vector2D v2, Transform2D t, std::string stroke){
        Point2D head{v1.x, v1.y};
        Point2D tail{v2.x, v2.y};
        Point2D newv1 = t(head);
        Point2D newv2 = t(tail);
        double x1 = newv1.x * 96.0 + 8.5/2.0* 96.0;
        double x2 = newv2.x * 96.0 + 8.5/2.0* 96.0;
        double y1 = -newv1.y * 96.0 + 11.0/2.0*96.0;
        double y2 = -newv2.y * 96.0 + 11.0/2.0*96.0;
        os << "<line x1=\""<< x1 << "\" x2=\""<< x2 << "\" y1=\""<< y1<< "\" y2=\"" << y2 <<"\" stroke=\"" << stroke << "\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />\n";
    }
    /// the x1 x2 y1 y2 here is from drawVector
    void groupTag(std::ostream & os, Transform2D t, std::string text){
        double x2 = t.translation().x* 96.0 + 8.5/2.0* 96.0;
        double offsety = -t.translation().y* 96.0 + 11.0/2.0*96.0;

        os << "<g>\n";
        drawVector(os, Vector2D{1, 0}, Vector2D{0, 0}, t, "red");
        drawVector(os, Vector2D{0, 1}, Vector2D{0, 0}, t, "green");
        os << "<text x=\""<< x2 << "\" y=\"" << offsety << "\">{" << text << "}</text>\n";
        os << "</g>\n";
    }
    void endSvg(std::ostream & os){
        os << "</svg>\n";
    }
}

    

