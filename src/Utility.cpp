#include <cmath>
#include <map>
#include <vector>
#include <utility>
#include <iostream>
#include <string>
#include <dirent.h>
#include "Utility.h"

namespace lane_model {

namespace utility {

// 0.001 mm = micrometer
double EPSILON = 1.0e-6;

void GetLine(const double x1, const double y1, const double x2, const double y2, double &a, double &b, double &c) {
  // (x- p1X) / (p2X - p1X) = (y - p1Y) / (p2Y - p1Y) 
  a = y2 - y1;
  b = x2 - x1;
  c = x1 * y2 - x2 * y1;
}

double safe_acos(double value) {
    if (value<=-1.0) {
        return M_1_PI;
    } else if (value>=1.0) {
        return 0;
    } else {
        return acos(value);
    }
}
double PerpendicularDistance(const Point &pt, const Point &lineStart, const Point &lineEnd)
{
	double dx = lineEnd.x - lineStart.x;
	double dy = lineEnd.y - lineStart.y;

	//Normalise
	double mag = pow(pow(dx,2.0)+pow(dy,2.0),0.5);
	if(mag > 0.0)
	{
		dx /= mag; dy /= mag;
	}

	double pvx = pt.x - lineStart.x;
	double pvy = pt.y - lineStart.y;

	//Get dot product (project pv onto normalized direction)
	double pvdot = dx * pvx + dy * pvy;

	//Scale line direction vector
	double dsx = pvdot * dx;
	double dsy = pvdot * dy;

	//Subtract this from pv
	double ax = pvx - dsx;
	double ay = pvy - dsy;

	return pow(pow(ax,2.0)+pow(ay,2.0),0.5);
}

void RamerDouglasPeucker(const std::vector<Point> &pointList, double epsilon, std::vector<Point> &out) {
	if(pointList.size()<2)
		throw std::invalid_argument("Not enough points to simplify");

	// Find the point with the maximum distance from line between start and end
	double dmax = 0.0;
	size_t index = 0;
	size_t end = pointList.size()-1;
	for(size_t i = 1; i < end; i++)
	{
		double d = PerpendicularDistance(pointList[i], pointList[0], pointList[end]);
		if (d > dmax)
		{
			index = i;
			dmax = d;
		}
	}

	// If max distance is greater than epsilon, recursively simplify
	if(dmax > epsilon)
	{
		// Recursive call
		std::vector<Point> recResults1;
		std::vector<Point> recResults2;
		std::vector<Point> firstLine(pointList.begin(), pointList.begin()+index+1);
		std::vector<Point> lastLine(pointList.begin()+index, pointList.end());
		RamerDouglasPeucker(firstLine, epsilon, recResults1);
		RamerDouglasPeucker(lastLine, epsilon, recResults2);
 
		// Build the result list
		out.assign(recResults1.begin(), recResults1.end()-1);
		out.insert(out.end(), recResults2.begin(), recResults2.end());
		if(out.size()<2)
			throw std::runtime_error("Problem assembling output");
	} 
	else 
	{
		//Just return start and end points
		out.clear();
		out.push_back(pointList[0]);
		out.push_back(pointList[end]);
	}
}

bool IsStringANumber(const std::string& s) {
    std::string::const_iterator it = s.begin();
    while (it != s.end() && std::isdigit(*it)) ++it;
    return !s.empty() && it == s.end();
}

double DistancePointTLineSegment(const Point point, const LineSegment line_segment) { 
  // Return minimum distance between line segment vw and point p
  // Consider the line extending the segment, parameterized as v + t (w - v).
	// i.e. |w-v|^2 -  avoid a sqrt
  const double squared_distance = line_segment.first.DistanceSquared(line_segment.second); 
  // find projection of point p onto the line. 
  const double t = std::max(0.0, std::min(1.0, utility::DotProduct(point - line_segment.first, line_segment.second - line_segment.first) / squared_distance));
	// Projection falls on the segment
  Point projection = line_segment.first + (line_segment.second - line_segment.first) * t;
	if (t >= 0 && t <= 1) {
		return point.Distance(projection);
	}
	else if (t < 0) {
		return point.Distance(line_segment.first);
	}
	else if (t > 1) {
		return point.Distance(line_segment.second);
	}
  //return perpendicular_distance = PerpendicularDistance(point, line_segment.first, line_segment.second);
}

double LineSegmentLength(const LineSegment line_segment) {
	return line_segment.first.Distance(line_segment.second);
}

double DotProduct(const Point point1, const Point point2) {
	return (point1.x * point2.x + point1.y * point2.y);
}

/* To find orientation of ordered triplet (p, q, r). 
   The function returns following values 
   0 --> p, q and r are colinear 
   1 --> Clockwise  right
   2 --> Counterclockwise left */
int TripletOrientation(Point p, Point q, Point r) 
{ 
    // See https://www.geeksforgeeks.org/orientation-3-ordered-points/ 
    // for details of below formula. 
    double val = (q.y - p.y) * (r.x - q.x) - 
              (q.x - p.x) * (r.y - q.y); 
  
    if (fabs(val) < 0.01) return 0;  // colinear 
  
    return (val > 0)? 1: 2; // clock or counterclock wise 
}

void CombinationUtil(std::vector<std::vector<int> >& ans, std::vector<int>& tmp, int n, int left, int k) { 
  // Pushing this vector to a vector of vector 
  if (k == 0) { 
    ans.push_back(tmp); 
    return; 
  } 
  // i iterates from left to n. First time 
  // left will be 1 
  for (int i = left; i <= n; ++i) 
  { 
    tmp.push_back(i); 
    CombinationUtil(ans, tmp, n, i + 1, k - 1); 
    // Popping out last inserted element 
    // from the vector 
    tmp.pop_back(); 
  } 
} 
  
/* 
	calculate all combinations of size k of numbers from 1 to n. 
	https://www.geeksforgeeks.org/print-all-possible-combinations-of-r-elements-in-a-given-array-of-size-n/
*/
std::vector<std::vector<int> > Combination(int n, int k) {
  std::vector<std::vector<int> > combinations; 
  std::vector<int> tmp; 
  CombinationUtil(combinations, tmp, n, 1, k); 
  return combinations; 
} 

double ConnectingAngleBwtweenLineSegment(const LineSegment line_segment1, const LineSegment line_segment2) {
	Point centerpoint1{(line_segment1.first.x + line_segment1.second.x)/2, (line_segment1.first.y + line_segment1.second.y)/2};
	Point centerpoint2{(line_segment2.first.x + line_segment2.second.x)/2, (line_segment2.first.y + line_segment2.second.y)/2};
  double distance_between_centers = centerpoint1.Distance(centerpoint2); 
	double distance_2_T_1 = PerpendicularDistance(centerpoint2, line_segment1.first, line_segment1.second);
  double connecting_angle = 0;
  if (distance_between_centers == 0.0) {
		double distance_21_T_1 = PerpendicularDistance(line_segment2.first, line_segment1.first, line_segment1.second);
		double length2 = line_segment2.first.Distance(line_segment2.second);
    connecting_angle = asin(distance_21_T_1 / length2) * 180 / M_PI;
  }
  else {
    connecting_angle = asin(distance_2_T_1 / distance_between_centers) * 180 / M_PI;
  }
	return connecting_angle;
}

double AngleBetweenTwoLineSegments(const LineSegment line_segment1, const LineSegment line_segment2) {
	double x1 = line_segment1.first.x - line_segment1.second.x;
	double y1 = line_segment1.first.y - line_segment1.second.y;
	double x2 = line_segment2.first.x - line_segment2.second.x;
	double y2 = line_segment2.first.y - line_segment2.second.y;
	// ???????
	double dot = sqrt(x1*x1 + y1*y1) * sqrt(x2*x2 + y2*y2);
	if (dot == 0.0) {
		return 0.0;
	}
	else {
		double angle = safe_acos((x1 * x2 + y1 * y2) / dot) ;
		return angle* 180 / M_PI;
	}
}

double DistanceBetweenTwoLineSegments(const LineSegment line_segment1, const LineSegment line_segment2) {
	double distance;
	distance = DistancePointTLineSegment(line_segment1.first, line_segment2);
	distance = std::min(distance, DistancePointTLineSegment(line_segment1.second, line_segment2));
	distance = std::min(distance, DistancePointTLineSegment(line_segment2.first, line_segment1));
	distance = std::min(distance, DistancePointTLineSegment(line_segment2.second, line_segment1));
	return distance;
}

/*
  Print functions
*/
void Print(std::string string) {
  std::cout << string << std::endl;
}

void PrintError(std::string string) {
  std::cout << "\033[1;31mERROR: " + string + "\033[0m" << std::endl;
}

void PrintWarning(std::string string) {
  std::cout << "\033[1;33mWarning: " + string + "\033[0m" << std::endl;
}

void PrintGreen(std::string string) {
  std::cout << "\033[1;32m" + string + "\033[0m" << std::endl;
}

} //ns utility

}// ns LANE_MODEL_UTILITY_H_