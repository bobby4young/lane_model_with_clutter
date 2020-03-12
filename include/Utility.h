#ifndef LANE_MODEL_UTILITY_H_
#define LANE_MODEL_UTILITY_H_

#include <string>
#include <vector>
#include "ObjectShape.h"

namespace lane_model {

namespace utility {

void GetLine(const double x1, const double y1, const double x2, const double y2, double &a, double &b, double &c);

double safe_acos(double value);

double PerpendicularDistance(const Point& pt, const Point& lineStart, const Point& lineEnd);

void RamerDouglasPeucker(const std::vector<Point> &pointList, double epsilon, std::vector<Point> &out);

bool IsStringANumber(const std::string& s);

double DistancePointTLineSegment(const Point point, const LineSegment line_segment);

double LineSegmentLength(const LineSegment line_segment);

double DotProduct(const Point point1, const Point point2);

int TripletOrientation(Point p, Point q, Point r);

std::vector<std::vector<int> > Combination(int n, int k);

/*
  the angle of line(line segment1) and line, which connects those two line segments' center point
*/
double ConnectingAngleBwtweenLineSegment(const LineSegment line_segment1, const LineSegment line_segment2);

double AngleBetweenTwoLineSegments(const LineSegment line_segment1, const LineSegment line_segment2);

double DistanceBetweenTwoLineSegments(const LineSegment line_segment1, const LineSegment line_segment2);
/*
  Print functions
*/
void Print(std::string string);

template <class T>
void Print(std::string string,  T value);

void PrintError(std::string string);

template <class T>
void PrintError(std::string string,  T value);

template <class T1, class T2>
void PrintError(std::string string,  T1 expected_value,  T2 value);

void PrintWarning(std::string string);

template <class T>
void PrintWarning(std::string string,  T value);

template <class T1, class T2>
void PrintWarning(std::string string,  T1 expected_value,  T2 value);

void PrintGreen(std::string string);

template <class T>
void PrintGreen(std::string string,  T value);

template <class T1, class T2>
void PrintGreen(std::string string,  T1 expected_value,  T2 value);
/*
  Print template functions definition
*/
template <class T>
void PrintWarning(std::string string,  T value) {
  std::cout << "\033[1;33mWarning: \033[0m" << string << " " << value << std::endl;
}

template <class T1, class T2>
void PrintWarning(std::string string,  T1 expected_value,  T2 value) {
  std::cout << "\033[1;33mWarning: \033[0m" << string << " should be " << expected_value << " and is " << value << std::endl;
}

template <class T>
void PrintGreen(std::string string,  T value) {
  std::cout << "\033[1;32mMessage: \033[0m" << string << " " << value << std::endl;
}

template <class T1, class T2>
void PrintGreen(std::string string,  T1 expected_value,  T2 value) {
  std::cout << "\033[1;32mMessage: \033[0m" << string << " should be " << expected_value << " and is " << value << std::endl;
}

template <class T>
void PrintError(std::string string,  T value) {
  std::cout << "\033[1;31mERROR: \033[0m" << string << " " << value << std::endl;
}

template <class T1, class T2>
void PrintError(std::string string,  T1 expected_value,  T2 value) {
  std::cout << "\033[1;31mERROR: \033[0m" << string << " should be " << expected_value << " and is " << value << std::endl;
}

template <class T>
void Print(std::string string,  T value) {
	std::cout << string << " " << value << std::endl;
}

} //ns utility

}// ns LANE_MODEL_UTILITY_H_
#endif