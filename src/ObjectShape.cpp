#include <algorithm>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <numeric>
#include "ObjectShape.h"
#include "Utility.h"

namespace lane_model {

double EPSILON = 1.0e-4;

Point::Point(double x, double y): cv::Point2d(x, y) {}

Point::Point(): cv::Point2d(0.0, 0.0) {}

Point::Point(const Point point1, const Point point2) {
  x = (point1.x + point2.x) / 2;
  y = (point1.y + point2.y) / 2;
}

Point::Point(const cv::Point2d cvpoint): cv::Point2d(cvpoint) {}

cv::Point2d Point::cvPoint() const {
	return cv::Point2d(x, y);
}

Point Point::operator + (Point point) const {
	Point temp;
	temp.x = x + point.x;
	temp.y = y + point.y;
	return temp;
}

Point Point::operator - (Point point) const {
	Point temp;
	temp.x = x - point.x;
	temp.y = y - point.y;
	return temp;
}

double Point::operator * (Point point) const {
	double dot_product = ((x * point.x) + (y * point.y));
	return dot_product;
}

Point Point::operator * (double scale) const {
	Point temp;
	temp.x = x * scale;
	temp.y = y * scale;
	return temp;
}

Point Point::operator * (int scale) const {
	Point temp;
	temp.x = x * scale;
	temp.y = y * scale;
	return (temp);
}

/*
	Another methode called ApproximatelyEqual considers points within 0.01m are equal
*/
bool Point::operator == (const Point point) const {
	if ((x == point.x) && (y == point.y)) return true;
	else return false;
}
// points within 0.01m are equal
bool Point::ApproximatelyEqual(const Point point) const {
	if ((std::abs(x - point.x) < 0.01) && (std::abs(y - point.y) < 0.01)) return true;
	else return false;
}

double Point::Distance(const Point point) const {
  return sqrt(pow(x - point.x, 2) + pow(y - point.y, 2));
}

double Point::ShortestDistance(const std::vector<Point> points) const {
  if (points.size() == 0) return 1.0e10;
	double shortest = Distance(points[0]);
  for(Point point : points){
    shortest = std::min(shortest, Distance(point));
  }
  return shortest;
}

double Point::DistanceSquared(Point point) const {
	return pow((x - point.x), 2) + pow((y - point.y), 2);
}

Box::Box(): center_(0, 0), height_(0), width_(0), orientation_(0) {}

Box::Box(cv::RotatedRect cv_rect): cv::RotatedRect(cv_rect) {
  center_.x = cv_rect.center.x;
  center_.y = cv_rect.center.y;
	if (cv_rect.size.height > cv_rect.size.width) {
		orientation_ = cv_rect.angle;
  	height_ = cv_rect.size.height;
  	width_ = cv_rect.size.width;
	}
	else {
		orientation_ = cv_rect.angle + 90.0;
  	height_ = cv_rect.size.width;
  	width_ = cv_rect.size.height;
	}
	if (width_ < 0.001) {
    width_ = 0.01;
  }
	if (height_ < 0.001) {
    height_ = 0.01;
  }
  cv::Point2f rect_points[4]; 
  cv_rect.points(rect_points);
  for (cv::Point2f point : rect_points) {
    vertices_.emplace_back(point.x, point.y);
  }
  if (vertices_.size() == 4 ) {
    vertices_valid = true;
    GenerateLine();
  }
  else {
    std::cout << "\033[1;31mbox vertices unvalid\033[0m" << std::endl;
    exit(1);
  }
}

double Box::ShortestDistanceToBox(const Box& box) const {
  assert(vertices_.size()==4);
  double shortest = DistanceBetweenBoxCenter(box);
  for(Point vertex : vertices_) {
    shortest = std::min(shortest, vertex.ShortestDistance(box.vertices_));
  }
  return shortest;
}

double Box::DistanceBetweenBoxCenter(const Box& box) const {
  return center_.Distance(box.center_);
}
// void Box::SetCenterX(std::string center_x_string){
//   double center_x = std::stof(center_xstring);
//   SetCenterX(center_x);
// }
// void Box::SetCenterX(double center_x){
//   center_.x = center_x;
// }
// void Box::SetCenterY(std::string center_y_string){
//   double center_y = std::stof(center_y_string);
//   SetCenterY(center_y);
// }
// void Box::SetCenterY(double center_y){
//   center_.y_ = center_y;
// }
// void Box::SetHeight(std::string height_string){
//   double height = std::stof(height_string);
//   SetHeight(height);
// }
// void Box::SetHeight(double height){
//   height_ = height;
// }
// void Box::SetWidth(std::string width_string){
//   double width = std::stof(width_string);
//   SetWidth(width);
// }
// void Box::SetWidth(double width){
//   width_ = width;
// }
// void Box::SetOrientation(std::string orientation_string){
//   double orientation = std::stof(orientation_string);
//   SetOrientation(orientation);
// }
// void Box::SetOrientation(double orientation){
//   orientation_ = orientation;
// }
double Box::GetHeight() const {
  return height_;
}
double  Box::GetWidth() const {
  return width_;
}

double Box::GetOrientation() const {
  return orientation_;
  //return angle;
}

LineSegment Box::GetLine() const {
	return line_segment_;
}

void Box::GenerateLine() {
	//TODO: move to constructor
  if (vertices_valid) {
    double distance_first_t_second = vertices_[0].Distance(vertices_[1]);
    double distance_first_t_fourth = vertices_[0].Distance(vertices_[3]);
    if (distance_first_t_second > distance_first_t_fourth) {
      Point point_1 = Point(vertices_[0], vertices_[3]);
      Point point_2 = Point(vertices_[1], vertices_[2]);
			line_segment_ = LineSegment(point_1, point_2);
    } 
		else {
      Point point_1 = Point(vertices_[0], vertices_[1]);
      Point point_2 = Point(vertices_[2], vertices_[3]);
			line_segment_ = LineSegment(point_1, point_2);
    }
  }
  else {
    std::cout << "box vertices unvalid when generating line" << std::endl;
  }
}

double Box::DistanceToPoint(Point point) const {
  if (vertices_valid) {
    // double a, b, c;
    // utility::GetLine(point_1_.x, point_1_.y, point_2_.x, point_2_.y, a, b, c);
		Point point_1 = line_segment_.first;
		Point point_2 = line_segment_.second;
    double a = point_2.y - point_1.y;
    double b = point_2.x - point_1.x;
    double c = point_1.x * point_2.y - point_2.x * point_1.y;
    return abs(a * point.x - b * point.y  - c) / sqrt(a * a + b * b);
  }
  else {
    std::cout << "box vertices unvalid" << std::endl;
    return 0.0;
  }
}

void Box::Merge(const Box& box) {
  std::vector<cv::Point2f> all_points;

  cv::Point2f points_1[4];
  points(points_1);
  if (points_1[0].x == 0 && points_1[0].y == 0 ) {
    *this = box;
    return;
  }
  all_points.push_back(points_1[0]);
  all_points.push_back(points_1[1]);
  all_points.push_back(points_1[2]);
  all_points.push_back(points_1[3]);

  cv::Point2f points_2[4];
  box.points(points_2);
  all_points.push_back(points_2[0]);
  all_points.push_back(points_2[1]);
  all_points.push_back(points_2[2]);
  all_points.push_back(points_2[3]);

  cv::RotatedRect cv_rect = cv::minAreaRect(all_points);
  

  if (cv_rect.size.width > cv_rect.size.height) {
    double width = cv_rect.size.height;
    double height = cv_rect.size.width;
    double angle = cv_rect.angle + 90.0;
    cv::RotatedRect cv_rect_corrected(cv_rect.center, {(float)width, (float)height}, angle);
    Box merged_box_corrected(cv_rect_corrected);
    *this = merged_box_corrected;
    return;
  }
  else {
    Box merged_box(cv_rect);
    *this = merged_box;
  }
}

void Box::Merge(const std::vector<Box>& boxes) {
  for (int i = 0; i < boxes.size(); i++) {
    Merge(boxes[i]);
  }
}

Point Box::Center() const {
  return center_;
}

double Box::CenterX() const {
  return center_.x;
}

double Box::CenterY() const {
  return center_.y;
}

/*
  describe vector for marking node edge is 1.ego.height 2.ego.width 3.other.height 4.other.width 5.dx 6.dy 7.angle
*/  
std::vector<double> Box::GetRelation(const Box& other) const {
	std::vector<double> relation;
	relation.push_back(height_);
	relation.push_back(width_);
	relation.push_back(orientation_);
	relation.push_back(other.height_);
	relation.push_back(other.width_);
	relation.push_back(other.orientation_);
	relation.push_back(center_.x - other.center_.x);
	relation.push_back(center_.y - other.center_.y);
	double angle = orientation_ - other.orientation_;
	if (angle > 90.0) {
		angle -= 90.0;
	}
	else if (angle < -90.0) {
		angle += 90.0;
	}
	//relation.push_back(angle);
	return relation;
}

// minimum distance between line segment vw and point p
double DistancePointToLineSegment(Point v, Point w, Point p) {
  const double squared_distance = v.DistanceSquared(w);  // i.e. |w-v|^2 -  avoid a sqrt
  if (squared_distance == 0.0) return p.Distance(v);   // v == w case
  /*Consider the line extending the segment, parameterized as v + t (w - v).
    We find projection of point p onto the line. 
    It falls where t = [(p-v) . (w-v)] / |w-v|^2
    We clamp t from [0,1] to handle points outside the segment vw. */
  const double t = std::max(0.0, std::min(1.0, utility::DotProduct(p - v, w - v) / squared_distance));
  // Projection falls on the line segment
  Point projection = v + (w - v) * t;  
  return p.Distance(projection);
}

float DistancePointToLineSegment(const LineSegment& line_segment, const Point& point) {
	return DistancePointToLineSegment(line_segment.first, line_segment.second, point);
}
  // Given three colinear points p, q, r, the function checks if point q lies on line segment 'pr' 
bool ColinearPointOnLineSegment(Point p, Point q, Point r) 
{ 
  if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) && 
  	q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y)) {
    return true;
	}
  return false; 
} 
  
// function that returns true if line segment 'p1q1' and 'p2q2' intersect. 
bool IsIntersect(const Point p1, const Point q1, const Point p2, const Point q2) 
{ 
  // Find the four orientations needed for general and 
  // special cases 
  int o1 = utility::TripletOrientation(p1, q1, p2); 
  int o2 = utility::TripletOrientation(p1, q1, q2); 
  int o3 = utility::TripletOrientation(p2, q2, p1); 
  int o4 = utility::TripletOrientation(p2, q2, q1); 

  // General case 
  if (o1 != o2 && o3 != o4) 
      return true; 

  // Special Cases 
  // p1, q1 and p2 are colinear and p2 lies on segment p1q1 
  if (o1 == 0 && ColinearPointOnLineSegment(p1, p2, q1)) return true; 

  // p1, q1 and q2 are colinear and q2 lies on segment p1q1 
  if (o2 == 0 && ColinearPointOnLineSegment(p1, q2, q1)) return true; 

  // p2, q2 and p1 are colinear and p1 lies on segment p2q2 
  if (o3 == 0 && ColinearPointOnLineSegment(p2, p1, q2)) return true; 

   // p2, q2 and q1 are colinear and q1 lies on segment p2q2 
  if (o4 == 0 && ColinearPointOnLineSegment(p2, q1, q2)) return true; 

  return false; // Doesn't fall in any of the above cases 
} 
bool IsLineSegmentIntersect(LineSegment line1, LineSegment line2) {
	Point p1 = line1.first;
	Point q1 = line1.second;
	Point p2 = line2.first;
	Point q2 = line2.second;
	return IsIntersect(p1, q1, p2, q2);
}

double PointDistance(const Point& point1, const Point& point2) {
  return sqrt(pow((point1.x-point2.x), 2) + pow((point1.y-point2.y), 2));
}

std::vector<Point> BoxSpine(cv::RotatedRect box) {
  std::vector<Point> points;
	cv::Point2f rect_points[4];
	box.points( rect_points );
	for (auto cvpoint: rect_points) {
		points.emplace_back(cvpoint);
	}
  std::vector<Point> four_points;
  four_points.emplace_back(points[0],  points[1]);
  four_points.emplace_back(points[1],  points[2]);
  four_points.emplace_back(points[2],  points[3]);
  four_points.emplace_back(points[3],  points[0]);
  std::vector<Point> outpoints;
  if (four_points[0].Distance(four_points[2]) > four_points[1].Distance(four_points[3])) {
    outpoints.push_back(four_points[0]);
    outpoints.push_back(four_points[2]);
  }
  else {
    outpoints.push_back(four_points[1]);
    outpoints.push_back(four_points[3]);
  }
  return outpoints;
}

Polyline::Polyline(): points_(0, Point()){}	

Polyline::Polyline(std::vector<Point> points): points_(points), length_sum_(0.0) {
	if (points.size() == 0 || points.size() == 1) {
  	std::cout << "Polyline needs least two points to construct" << std::endl; 
		return;
	} 
	middle_points_.reserve(points_.size()-1);
	line_segments_.reserve(points_.size()-1);

	// TODO Sorting in this way could potentially create bugs when the lines are near horizontal
	// first point's x is smaller than last point's x
	if (points.back().y >= points.front().y) {
		for (int i = 0; i < (points_.size()-1); i++) {
			double x = (points_[i].x + points_[i+1].x) / 2.0;
			double y = (points_[i].y + points_[i+1].y) / 2.0;
			middle_points_.emplace_back(x, y);
			line_segments_.emplace_back(points_[i], points_[i+1]);
			double length = PointDistance(points_[i], points_[i+1]);
			line_length_.push_back(length);
			length_sum_ += length;
		}
	}
	else {
		// points_'s x in sorted order
		std::reverse(points_.begin(), points_.end());
		for (int i = 0; i < (points_.size()-1); i++) {
			double x = (points_[i].x + points_[i+1].x) / 2.0;
			double y = (points_[i].y + points_[i+1].y) / 2.0;
			middle_points_.emplace(middle_points_.begin(), x, y);
			line_segments_.emplace(line_segments_.begin(), points_[i], points_[i+1]);
			double length = PointDistance(points_[i], points_[i+1]);
			line_length_.emplace(line_length_.begin(), length);
			length_sum_ += length;
		}
	}
}

Polyline::Polyline(LineSegment line_segment): points_({line_segment.first, line_segment.second}) {
	double x = (points_[0].x + points_[1].x) / 2.0;
	double y = (points_[0].y + points_[1].y) / 2.0;
	middle_points_.emplace_back(x, y);
  length_sum_ = line_segment.first.Distance(line_segment.second);
	line_length_.push_back(length_sum_);
	// first point's x is larger than last point's x
	if (line_segment.first.y >= line_segment.second.y) {
		std::reverse(points_.begin(), points_.end());
	}
	line_segments_.emplace_back(points_[0], points_[1]);
}

int Polyline::NPoints() const {
	return points_.size();
}

int Polyline::NLines() const {
	return line_segments_.size();
}

bool Polyline::operator == (Polyline polyline) const {
	for (auto point1: points_) {
		for (auto point2: polyline.points_) {
			if (!(point1 == point2)) {
				return false;
			}
		}
	}
	return true;
}

std::vector<LineSegment> Polyline::GetLineSegments() const {
	return line_segments_;
}

std::vector<Point> Polyline::GetPoints() const {
	return points_;
}

LineSegment Polyline::ClosestLineSegment(const Point point) const {
	std::vector<Point>::const_iterator point_it = points_.begin();
	LineSegment closest_line = LineSegment(*point_it, *(point_it + 1));
	double min_distance = utility::DistancePointTLineSegment(point, closest_line);
	for (point_it = points_.begin() + 1; point_it != (points_.end() - 1); point_it ++) {
		LineSegment line = LineSegment(*point_it, *(point_it + 1));
		double distance = utility::DistancePointTLineSegment(point, line);
		if (distance < min_distance) {
			min_distance = distance;
			closest_line = line;
		}
	}
	return closest_line;
}

LineSegment Polyline::StraightLine() const {
	return LineSegment(points_.front(), points_.back());
}

double Polyline::AngleBetweenTwoLineSegments(LineSegment line_segment1, LineSegment line_segment2) const {
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
		double angle = utility::safe_acos((x1 * x2 + y1 * y2) / dot) ;
		//std::cout << " angle " << angle* 180 / M_PI << std::endl;
		return angle* 180 / M_PI;
	}
}

double Polyline::AngleBetweenTwoLineSegments(Point point1, Point point2, Point point3) const {
	LineSegment line_segment1{point1, point2};
	LineSegment line_segment2{point2, point3};
	return AngleBetweenTwoLineSegments(line_segment1, line_segment2);
}

double Polyline::AngleBetweenPolyline(const Polyline polyline) const {
	double angle = 0.0;
	for (std::vector<Point>::const_iterator it = points_.begin(); it != points_.end() - 1; it ++) {
		Point middle_point = Point(*it, *(it + 1));
		LineSegment closest_line = polyline.ClosestLineSegment(middle_point);
		LineSegment line{*it, *(it + 1)};
		double line_length = (*it).Distance(*(it + 1));
		angle += AngleBetweenTwoLineSegments(line, closest_line) * line_length / length_sum_;
	}
	if (angle > 90.0) {
		angle -= 90;
	}
	return angle;
}

double Polyline::CurveAngle() const {
	double angle = 0.0;
	if (points_.size() < 3) {
		return angle;
	}
	// if we only have three points
	for(std::vector<Point>::const_iterator it = points_.begin(); it != points_.end()-2; it ++) {
		angle += std::abs(AngleBetweenTwoLineSegments(*it, *(it+1), *(it+2)));
	}
	// altanative way is don't average to n angle
	angle = angle / (NPoints() - 2);
	return angle; 
}

std::vector<double> Polyline::CurveAngles() const {
	if (points_.size() == 2) {
		std::vector<double> two_points(1, 0.0);
		return two_points;
	}
	std::vector<double> angles;
	for(std::vector<Point>::const_iterator it = points_.begin(); it != points_.end()-2; it ++) {
		angles.push_back(AngleBetweenTwoLineSegments(*it, *(it+1), *(it+2)));
	}
	return angles; 
}

double Polyline::CurveAngleMax() const {
	double max_angle = 0.0;
	if (points_.size() < 3) {
		return max_angle;
	}
	std::vector<Point>::const_iterator it = points_.begin();
	max_angle = std::abs(AngleBetweenTwoLineSegments(*it, *(it+1), *(it+2)));
	for(it+1; it != points_.end()-2; it ++) {
		max_angle = std::max(std::abs(AngleBetweenTwoLineSegments(*it, *(it+1), *(it+2))), max_angle);
	}
	return max_angle; 
}

bool Polyline::CheckIntersect(LineSegment input_line) const {
	for (auto line : line_segments_) {
		if (IsLineSegmentIntersect(line, input_line)) {
			return true;
		}
	}
	return false;
}

bool Polyline::CheckIntersect(Polyline polyline) const {
	for (auto line : line_segments_) {
		for (auto other_line : polyline.GetLineSegments()) {
			if (IsLineSegmentIntersect(line, other_line)) {
				return true;
			}
		}
	}
	return false;
}

double Polyline::DistanceTPoint(const Point point) const {
	LineSegment line_segment = LineSegment{points_.front(), points_.at(1)};
	double min_distance = utility::DistancePointTLineSegment(point, line_segment);
	for (std::vector<Point>::const_iterator point_it = points_.begin() + 1; point_it != points_.end() - 1; point_it ++) {
		LineSegment line_segment = LineSegment{*point_it, *(point_it + 1)};
		double distance = utility::DistancePointTLineSegment(point, line_segment);
		min_distance = std::min(min_distance, distance);
	}
	return min_distance;
}

double Polyline::DistanceBetweenPolyline(const Polyline polyline) const {
	double distance = 0.0;
	if (length_sum_ <= polyline.GetLength()) {
		for (auto point: points_) {
			distance += polyline.DistanceTPoint(point);
		}
		distance = distance / NPoints();
	}
	else {
		std::vector<Point> points = polyline.GetPoints();
		for (auto point: points) {
			distance += DistanceTPoint(point);
		}
		distance = distance / polyline.NPoints();
	}
	return distance;
}

double Polyline::MaxDistanceBetweenPolyline(const Polyline polyline) const {
	double max_distance;
	if (length_sum_ <= polyline.GetLength()) {
		max_distance = polyline.DistanceTPoint(points_.front());
		for (auto point: points_) {
			max_distance = std::max(max_distance, polyline.DistanceTPoint(point));
		}
	}
	else {
		std::vector<Point> points = polyline.GetPoints();
		max_distance = DistanceTPoint(points.front());
		for (auto point: points) {
			max_distance = std::max(max_distance, DistanceTPoint(point));
		}
	}
	return max_distance;
}


double Polyline::MinDistanceBetweenPolyline(const Polyline polyline) const {
	double min_distance = polyline.DistanceTPoint(points_.front());
	for (auto point: points_) {
		min_distance = std::min(min_distance, polyline.DistanceTPoint(point));
	}
	std::vector<Point> points = polyline.GetPoints();
	for (auto point: points) {
		min_distance = std::min(min_distance, DistanceTPoint(point));
	}
	return min_distance;
	// LineSegment line_segment = polyline.ClosestLineSegment(points_.front());
	// double min_distance = utility::DistancePointTLineSegment(points_.front(), line_segment);
	// for (std::vector<Point>::const_iterator point_it = points_.begin() + 1; point_it != points_.end(); point_it++) {
	// 	line_segment = polyline.ClosestLineSegment(*point_it);
	// 	double distance = utility::DistancePointTLineSegment(*point_it, line_segment);
	// 	if (distance < min_distance) {
	// 		min_distance = distance;
	// 	}
	// }
	// return min_distance;
}

Point Polyline::GetDirection() const {
	// assuming x >= 0
	Point direction(points_.back() - points_.front());
	return direction;
}

Point Polyline::GetCenterPoint() const {
	return Point(points_.front(), points_.back());
}

Point Polyline::GetMiddlePoint() const {
	double traveled_length = 0.0;
	std::vector<Point>::const_iterator it;
	for (it = points_.begin(); it != points_.end() - 1; it ++) {
		traveled_length += (*it).Distance(*(it + 1));
		if (traveled_length > (length_sum_ / 2)) {
			break;
		}
	}
	Point point1 = *it;
	Point point2 = *(it + 1);
	double distance1t2 = point1.Distance(point2);
	// traveled_length - ratio * vector2t1 = length_sum_ / 2
	double ratio = (traveled_length - (length_sum_ / 2)) / distance1t2;
	Point vector2t1 = point1 - point2;
	Point middle_point = point2 + vector2t1 * ratio;
	return middle_point;
}

double Polyline::GetLength() const {
	// double length = 0.0;
	// for (std::vector<Point>::const_iterator point_it = points_.begin(); point_it != points_.end() - 1;point_it ++) {
	// 	length += (*point_it).Distance(*(point_it + 1));
	// }
	// //return length;
	// if (length != length_sum_) {
	// 	utility::PrintError("length ", std::to_string(length) + " " + std::to_string(length_sum_));
	// }
	return length_sum_;
}

Polyline Polyline::Reverse() const {
	std::vector<Point> points(points_);
	std::reverse(points.begin(), points.end());
	return Polyline(points);
}

Polyline Polyline::GetCenterLine(Polyline polyline) const {
	std::vector<Point> points;
	Point first_point(points_.front(), polyline.points_.front());
	Point last_point(points_.back(), polyline.points_.back());
	if (polyline.NPoints() == 2 && NPoints() == 2) {
		points.push_back(first_point);
		points.push_back(last_point);
		return Polyline(points);
	}
	else if (polyline.NPoints() >= NPoints()) {
		// project polyline to this
		points.push_back(first_point);
		for (std::vector<Point>::iterator point_it = polyline.points_.begin() + 1; point_it != polyline.points_.end() - 1; point_it++) {
			LineSegment line_segment = ClosestLineSegment(*point_it);
  		// Return minimum distance between line segment vw and point p
  		// Consider the line extending the segment, parameterized as v + t (w - v).
			// i.e. |w-v|^2 -  avoid a sqrt
  		const double squared_distance = line_segment.first.DistanceSquared(line_segment.second); 
  		// find projection of point p onto the line. 
  		const double t = std::max(0.0, std::min(1.0, utility::DotProduct(*point_it - line_segment.first, line_segment.second - line_segment.first) / squared_distance));
  		Point projection = line_segment.first + (line_segment.second - line_segment.first) * t;  // Projection falls on the segment
			points.emplace_back(*point_it, projection);
		}
		points.push_back(last_point);
	}
	else {
		// project this to polyline
		points.push_back(first_point);
		for (std::vector<Point>::const_iterator point_it = points_.begin() + 1; point_it != points_.end() - 1; point_it++) {
			LineSegment line_segment = polyline.ClosestLineSegment(*point_it);
  		const double squared_distance = line_segment.first.DistanceSquared(line_segment.second); 
  		const double t = std::max(0.0, std::min(1.0, utility::DotProduct(*point_it - line_segment.first, line_segment.second - line_segment.first) / squared_distance));
  		Point projection = line_segment.first + (line_segment.second - line_segment.first) * t;
			points.emplace_back(*point_it, projection);
		}
		points.push_back(last_point);
	}
	return Polyline(points);
}

int Polyline::LeftOrRight(const Polyline polyline) const {
	LineSegment line_segment = ClosestLineSegment(polyline.points_.front());
	int direction = utility::TripletOrientation(line_segment.first, line_segment.second, polyline.points_.front());
	for (std::vector<Point>::const_iterator it = polyline.points_.begin() + 1; it != polyline.points_.end(); it++) {
	  line_segment = ClosestLineSegment(*it);
	  if(utility::TripletOrientation(line_segment.first, line_segment.second, *it) != direction) {
			// points from polyline has different orientation return ceter point orientation
			return 3 + utility::TripletOrientation(StraightLine().first, StraightLine().second, *it);
		}
	}
	return direction;
}

int Polyline::PointLeftOrRight(const Point point) const {
	LineSegment line_segment = ClosestLineSegment(point);
	int direction = utility::TripletOrientation(line_segment.first, line_segment.second, point);
	for (std::vector<Point>::const_iterator it = points_.begin(); it != points_.end() - 1 ; it++) {
	  line_segment = LineSegment(*it, *(it + 1));
	  if(utility::TripletOrientation(line_segment.first, line_segment.second, point) != direction) {
			// points from polyline has different orientation return ceter point orientation
			return 3 + utility::TripletOrientation(StraightLine().first, StraightLine().second, point);
		}
	}
	return direction;
}

void Polyline::InsertPointPushback(const Point& point) {
	points_.push_back(point);
  //RamerDouglasPeucker(points_, 20, points_);
	int last_index = NPoints() - 1;
	double x = (points_[last_index].x + point.x) / 2.0;
	double y = (points_[last_index].y + point.y) / 2.0;
	middle_points_.emplace_back(x, y);
	line_segments_.emplace_back(points_[last_index], point);
	double length = PointDistance(points_[last_index], point);
	line_length_.push_back(length);
	length_sum_ += length;
}
void Polyline::InsertPointSort(const Point& point) {
	// TODO: 
}
void Polyline::InsertPointAfter(const Point& point, std::vector<Point>::iterator anchor_point) {
	// Check the point should be inserted in the middle or appended at the beginning or end
  if (NPoints() == 0 || NPoints() == 1) {
  	std::cout << "Can't build polyline from one point" << std::endl; 
		exit(1);
	}
  int index = anchor_point - points_.begin();
	if (anchor_point == points_.begin()) {
		double x = (point.x + points_[0].x) / 2.0;
	 	double y = (point.y + points_[0].y) / 2.0;
		middle_points_.emplace(middle_points_.begin(), x, y);

		double length = PointDistance(point, points_.front());
		line_segments_.emplace(line_segments_.begin(), point, points_.front());
		line_length_.insert(line_length_.begin(), length);
		length_sum_ += length;
	}
	else if (anchor_point == points_.end()) {
		double x = (point.x + points_.back().x) / 2.0;
	 	double y = (point.y + points_.back().y) / 2.0;
		middle_points_.emplace(middle_points_.end(), x, y);

		double length = PointDistance(point, points_.back());
		line_segments_.emplace(line_segments_.end(), points_.back(), point);
		line_length_.insert(line_length_.end(), length);
		length_sum_ += length;
	}
	else {
		// insert the point between anchor_point and adjacent_point
		int index = anchor_point - points_.begin() - 1; // index >= 0
		std::vector<Point>::iterator adjacent_point = anchor_point - 1;
		double x1 = (point.x + adjacent_point->x) / 2.0;
	 	double y1 = (point.y + adjacent_point->y) / 2.0;
		double x2 = (point.x + anchor_point->x) / 2.0;
	 	double y2 = (point.y + anchor_point->y) / 2.0;
		middle_points_.erase(middle_points_.begin() + index);
		middle_points_.emplace(middle_points_.begin()+ index, x2, y2);
		middle_points_.emplace(middle_points_.begin()+ index, x1, y1);

		line_segments_.erase(line_segments_.begin() + index);
		line_segments_.emplace(line_segments_.begin() + index, point, *anchor_point);
		line_segments_.emplace(line_segments_.begin() + index, *adjacent_point, point);
		
		double length1 = PointDistance(point, *adjacent_point);
		double length2 = PointDistance(point, *anchor_point);
		//length_sum_ -= *(line_length_.begin() + index);
		line_length_.erase(line_length_.begin() + index);
		line_length_.insert(line_length_.begin() + index, length1);
		line_length_.insert(line_length_.begin() + index, length2);
		//length_sum_ += length1;
		//length_sum_ += length2;
		length_sum_ = Polyline(points_).GetLength();
	}
	points_.insert(anchor_point, point);
}

void Polyline::InsertPointInsert(const Point& input_point) {
	// Check the point should be inserted in the middle or appended at the beginning or end
  if (NPoints() == 0 || NPoints() == 1) {
  	std::cout << "Can't build polyline from one point" << std::endl; 
		exit(1);
	}
	int first_index = 0, second_index = 1;
	/* 
		point1 is the closest point to input_point in points_ 
		point2 is the sescond closest point to input_point in points_ 
		point3 is the closest point to point1 in direction of point1 to point2
		because point1 and poin2 might not be adjacent in points_ array
	*/
	std::vector<Point>::iterator point1;
	std::vector<Point>::iterator point2;
	std::vector<Point>::iterator point3;
	double first_distance = points_[0].DistanceSquared(input_point);
	double second_distance = points_[1].DistanceSquared(input_point);
	if (first_distance > second_distance) {
		first_distance = second_distance;
		second_distance = input_point.DistanceSquared(points_[0]);
		point1 = points_.begin() + 1;
		point2 = points_.begin();
	}
	else {
		point1 = points_.begin();
		point2 = points_.begin() + 1;
	}
	if (NPoints() > 2) {
		// Closest might be identical to second closest
		for (std::vector<Point>::iterator point_it = points_.begin() + 2; point_it != points_.end(); point_it ++) {
			double distance = point_it->DistanceSquared(input_point);
			if (distance < first_distance + EPSILON) {
				second_distance = first_distance;
				first_distance = distance;
				point2 = point1;
				point1 = point_it;
			}
			else if (distance > first_distance - EPSILON && distance < second_distance - EPSILON) {
				second_distance = distance;
				point2 = point_it;
			}
		}
	}	
	int offset = 0;
	// if the closest point's index is smaller than the second closest point's index
	if (std::distance(point1, point2) > 0) {
		// point1 is garanteed not to be points_.back()
		point3 = point1 + 1;
	}
	else {
		// point1 is garanteed not to be points_.begin() because existence of point2
		point3 = point1 - 1;
	}
	const double ratio = utility::DotProduct(input_point - *point1, *point3 - *point1) /  point1->DistanceSquared(*point3);
	if (std::distance(point1, point3) > 0) {
		offset = 1;
	}
	if (ratio >= 0 &&  ratio < 1) {
		// point is between point1 and point2
		InsertPointAfter(input_point, point1 + offset);
	}
	else if (ratio < 0) {
		InsertPointAfter(input_point, point1 - offset + 1);
	}
	else {
    utility::PrintWarning( "In InsertPointInsert, ratio is lager than 1, shouldn't happen ");
	}
}
void Polyline::InsertPointAltnative(const Point& input_point) {
	// Check the point should be inserted in the middle or appended at the beginning or end
  if (NPoints() == 0 || NPoints() == 1) {
  	std::cout << "Can't build polyline from one point" << std::endl; 
		exit(1);
	}
	int first_index = 0, second_index = 1;
	std::vector<Point>::iterator point1;
	std::vector<Point>::iterator point2;
	double min_distance = DistancePointToLineSegment(line_segments_[0], input_point);
	std::vector<LineSegment>::iterator closest_line_segment = line_segments_.begin();
	for (std::vector<LineSegment>::iterator line_segment_it = line_segments_.begin(); line_segment_it != line_segments_.end();
			line_segment_it++) {
		double distance = DistancePointToLineSegment(*line_segment_it, input_point);
  	std::cout << "distance " << distance << std::endl; 
		if (distance < min_distance) {
			min_distance = distance;
  		std::cout << "min distance " << min_distance << std::endl; 
			closest_line_segment = line_segment_it;
		}
	}
  std::cout << "index min line segment " << closest_line_segment - line_segments_.begin() << std::endl; 
  int index = closest_line_segment - line_segments_.begin();
	double first_distance = input_point.DistanceSquared(closest_line_segment->first);
	double second_distance = input_point.DistanceSquared(closest_line_segment->second);
  std::cout << "first_distance " << first_distance << std::endl; 
	if (first_distance > second_distance) {
		point1 = points_.begin() + index + 1;
		point2 = points_.begin() + index;
		
  	std::cout << "Swap at beginning" << std::endl; 
	}
	else {
		point1 = points_.begin() + index;
		point2 = points_.begin() + index + 1;
  	std::cout << "no Swap at beginning" << std::endl; 
	}
	std::cout << "initialized first point " << point1->x << " " << point1->y << std::endl;
	std::cout << "initialized second point " << point2->x <<" " << point2->y << std::endl;

	int direction = std::distance(point1, point2);
	std::cout << "direction " << direction << std::endl;
	
	int offset = 0;
	// if the closest point's index is smaller than the second closest point's index
	if (std::distance(point1, point2) > 0) {
		offset = 1;
	}
	const double ratio = utility::DotProduct(input_point - *point1, *point2 - *point1) /  point1->DistanceSquared(*point2);
	std::cout << "ratio is " << ratio << std::endl;

	if (ratio >= 0 &&  ratio < 1) {
		// point is between point1 and point2
		InsertPointAfter(input_point, point1 + offset);
	}
	else if (ratio < 0) {
		InsertPointAfter(input_point, point1 - offset + 1);
	}
	else {
		std::cout << "ratio is lager than 1, shouldn't happen " << ratio << std::endl;
	}
	std::cout << "n points after " << NPoints() << std::endl;
	for (auto point: points_) {
		std::cout << "point " << point.x << " " << point.y << std::endl;
	}
}

void Polyline::InsertPoint(const Point& point, int insert_option) {
	for (auto p: points_) {
		if (point.ApproximatelyEqual(p)) {
			//utility::PrintWarning("Polyline trying to insert a point, which is already in the polyline");
			return;
		}
	}
	switch(insert_option) {
		case 1: InsertPointPushback(point); break;
		case 2: InsertPointSort(point); break;
		case 3: InsertPointInsert(point); break;
		case 4: InsertPointAltnative(point); break;
		default: std::cout << "Polyline insert option " << insert_option << " is unvalid"<< std::endl;
	}
}

std::pair<Polyline, Polyline> Polyline::SplitIntoTwoPolylines(const Point split_point) const {
	// assume point is on this polyline
	LineSegment first_line_segment = line_segments_.front();
	float distance = DistancePointToLineSegment(first_line_segment, split_point);
	for (auto line: line_segments_) {
		distance = std::min(DistancePointToLineSegment(line, split_point), distance);
	}
	if (distance > 0.1) {
		std::cerr << "polyline is trying to splitting with a point outside of polyline" << std::endl;
		std::cerr << "Distance is " << distance << std::endl;
		exit(2);
	}
	std::vector<Point>::const_iterator it;
	//int count =0;
	double distance_point_to_segments;
	for(it = points_.begin(); it != points_.end(); ++it) {
		//count++;
		LineSegment line_segment{*it, *(it + 1)};
		distance_point_to_segments = DistancePointToLineSegment(line_segment, split_point);
		//std::cout << " Distance point to line segments " << distance_point_to_segments << std::endl;
		if (distance_point_to_segments < 1.0e-2) {
			break;
		}
	}
	if (distance_point_to_segments >= 1.0e-2) {
		std::cout  << std::endl << "returning one polyline after splitting split_point is " << split_point.x << " " << split_point.y << std::endl;
		std::cout << "points_ has " << points_.size() << std::endl;
		for (auto p: points_) {
			std::cout << p.x << " " << p.y << std::endl;
		}
		return std::pair<Polyline, Polyline>{*this, Polyline()};
	}
	std::vector<Point> points1(points_.begin(), it + 1);
	points1.push_back(split_point);

	std::vector<Point> points2(1, split_point);
	points2.insert(points2.end(), it + 1, points_.end());
	
	std::cout  << std::endl << "split_point is " << split_point.x << " " << split_point.y << std::endl;
	std::cout << "points1 has " << points1.size() << std::endl;
	for (auto p: points1) {
		std::cout << p.x << " " << p.y << std::endl;
	}
	std::cout << "points2 has " << points2.size() << std::endl;
	for (auto p: points2) {
		std::cout << p.x << " " << p.y << std::endl;
	}
  Polyline polyline1(points1);
  Polyline polyline2(points2);

	return std::pair<Polyline, Polyline>{polyline1, polyline2};
}

Point Polyline::GetPointWithPortion(double portion) const {
	if (portion < 0.0 || portion >= 1.0) {
		utility::PrintError("unvalid portion in GetPointWithPortion in Polyline");
	}
	const double aim_length = portion * length_sum_;
	double traveled_distance = 0.0;
	std::vector<Point>::const_iterator point_it;
	for (point_it = points_.begin(); point_it != (points_.end() - 1); point_it ++) {
		traveled_distance += point_it->Distance(*(point_it + 1));
		if (traveled_distance > aim_length) {
			break;
		}
	}
	double line_segment_length = point_it->Distance(*(point_it + 1));
	traveled_distance -= line_segment_length;
	double ratio = (aim_length - traveled_distance) / line_segment_length;
	Point point = *point_it + (*(point_it + 1) - *point_it) * ratio;
	return point;
}

std::vector<double> Polyline::GetRelation(const Polyline& other) const {
	std::vector<double> relation;
	relation.reserve(32);
	relation.push_back(length_sum_);
	relation.push_back(other.length_sum_);
	for (int i = 0; i < 30; i ++) {
		double distance = DistanceTPoint(other.GetPointWithPortion((double)i / 30.0));
		relation.push_back(distance);
	}
	return relation;
}

bool IntersectionPointOfTwoLineSegment(LineSegment line_segment1, LineSegment line_segment2, Point& intersection_point){
  if (!IsLineSegmentIntersect(line_segment1, line_segment2)) {
		return false;
	}
	/*
		refer to
		http://www.cs.swan.ac.uk/~cssimon/line_intersection.html
	*/
	Point point1 = line_segment1.first;
  Point point2 = line_segment1.second;
  Point point3 = line_segment2.first;
  Point point4 = line_segment2.second;
	double y12 = point1.y - point2.y;
	double y13 = point1.y - point3.y;
	double y34 = point3.y - point4.y;
	double x12 = point1.x - point2.x;
	double x13 = point1.x - point3.x;
	double x43 = point4.x - point3.x;
	double ta = (y34*x13 + x43*y13) / (x43*y12 + x12*y34);
	intersection_point = point1 + (point2 - point1) * ta;
}

Polygon::Polygon() {
	complete_ = false;
}

Polygon::Polygon(Polyline polyline) {
	left_line_ = polyline;
	complete_ = false;
}


Polygon::Polygon(Polyline polyline1, Polyline polyline2) {
	if (polyline1.CheckIntersect(polyline2)) {
    utility::PrintError("constructor of Polygon has two intersecting polylines");
	}
	if ((polyline1.GetDirection() * polyline2.GetDirection()) < 0) {
    utility::PrintError("constructor of Polygon has two opposite polylines");
	}
  int position = polyline1.LeftOrRight(polyline2);
	if (position == 1 || position == 4) {
		// 2 is on the right of 1
		left_line_ = polyline1;
		right_line_ = polyline2;
		complete_ = true;
	}
  else if (position == 2 || position == 5) {
		// 1 is on the right of 2
		left_line_ = polyline2;
		right_line_ = polyline1;
		complete_ = true;
	}
	else {
    utility::PrintError("Polygon is trying to add one line, position code:", position);
		complete_ = false;
	}
	if (complete_) {
		centerline_ = left_line_.GetCenterLine(right_line_);
		centerpoint_ = centerline_.GetMiddlePoint();
		width_ = left_line_.DistanceBetweenPolyline(right_line_);
		std::string point_coordinate = std::to_string(centerpoint_.x) + " " + std::to_string(centerpoint_.y);
		if (left_line_.PointLeftOrRight(centerpoint_) == 2 || right_line_.PointLeftOrRight(centerpoint_) == 1) {
    	utility::PrintWarning("center point is out of polygon:", point_coordinate);			
		}
	}
}

Polyline Polygon::GetLeftline() const {
	return left_line_;
}

Polyline Polygon::GetRightline() const {
	return right_line_;
}

Polyline Polygon::GetCenterLine() const {
	return centerline_;
}

Point Polygon::GetCenterPoint() const {
	return centerpoint_;
}

double Polygon::GetWidth() const {
	return width_;
}

double Polygon::GetWidthAtPoint(const Point point) const {
	// Get the closest lline segment to work with curved polyline
	LineSegment left_line = left_line_.ClosestLineSegment(point);
	LineSegment right_line = right_line_.ClosestLineSegment(point);
	if (utility::TripletOrientation(left_line.first, left_line.second, point) == 2 ||
	  utility::TripletOrientation(right_line.first, right_line.second, point) == 1) {
		std::string point_coordinate = std::to_string(point.x) + " " + std::to_string(point.y);
	  //utility::PrintWarning("get width function in Polygon has a point outside of polygon", point_coordinate);
		return INT_MAX;
	}
	double distance = left_line_.DistanceTPoint(point);
	distance += right_line_.DistanceTPoint(point);
	return distance;
}

bool Polygon::IsValid() const {
	return complete_;
}

bool Polygon::AddPolyline(Polyline polyline) {
	if (complete_) {
    utility::PrintError("Polygon is trying to add an extra line");
		return false;
	}
	if (left_line_.NPoints() == 0) {
		left_line_ = polyline;
	  utility::PrintWarning("Polygon::AddPolyline was called when polygon has no polyline at all");		
		return false;
	}
  int position = left_line_.LeftOrRight(polyline);
	if (position == 1 || position == 4) {
		// input is on the right of left_line_
		right_line_ = polyline;
		complete_ = true;
	}
  else if (position == 2 || position == 5) {
		// input is on the left of left_line
		right_line_ = left_line_;
		left_line_ = polyline;
		complete_ = true;
	}
	else {
    utility::PrintError("Polygon is trying to add one line, position code:", position);
		complete_ = false;
	}
	if (complete_) {
		centerline_ = left_line_.GetCenterLine(right_line_);
		centerpoint_ = centerline_.GetMiddlePoint();
		//centerpoint_ = centerline_.GetCenterPoint();
		width_ = left_line_.DistanceBetweenPolyline(right_line_);
		std::string point_coordinate = std::to_string(centerpoint_.x) + " " + std::to_string(centerpoint_.y);    	
		if (left_line_.PointLeftOrRight(centerpoint_) == 2 || right_line_.PointLeftOrRight(centerpoint_) == 1) {
			utility::PrintWarning("center point is out of polygon:", point_coordinate);			
		}
		if ((left_line_.GetDirection() * right_line_.GetDirection()) < 0) {
  	  utility::PrintError("AddPolyline() of Polygon has two opposite polylines");
			return false;
		}
		return true;
	}
	return false;
}

void Polygon::MergeWithPolyline(Polyline polyline) {
	if (!complete_) {
    utility::PrintError("uncomplete Polygon is trying to merge");
		return;
	}
	if (left_line_ == polyline || right_line_ == polyline) {
		return;
	}
  int position_to_left = left_line_.LeftOrRight(polyline);
  int position_to_right = right_line_.LeftOrRight(polyline);
	if (position_to_right == 1 || position_to_right == 4) {
		// input is on the right of right_line_
		right_line_ = polyline;
	}
  else if (position_to_left == 2 || position_to_left == 5){
		// input is on the left of left_line
		left_line_ = polyline;
	}
	else {
    //utility::PrintWarning("Polygon is trying to add one line, which is inside the polygon");
	}
	if (complete_) {
		centerline_ = left_line_.GetCenterLine(right_line_);
		//centerpoint_ = centerline_.GetCenterPoint();
		centerpoint_ = centerline_.GetMiddlePoint();
		width_ = left_line_.DistanceBetweenPolyline(right_line_);
		std::string point_coordinate = std::to_string(centerpoint_.x) + " " + std::to_string(centerpoint_.y);
		if (left_line_.PointLeftOrRight(centerpoint_) == 2 || right_line_.PointLeftOrRight(centerpoint_) == 1) {
			utility::PrintWarning("center point is out of polygon:", point_coordinate);			
		}
		if ((left_line_.GetDirection() * right_line_.GetDirection()) < 0) {
    	utility::PrintError("AddPolyline() of Polygon has two opposite polylines");
		}
	}
}

void Polygon::MergeWithPolygon(Polygon polygon) {
	Polyline left_line = polygon.GetLeftline();
	MergeWithPolyline(left_line);
	Polyline right_line = polygon.GetRightline();
	MergeWithPolyline(right_line);
}

}//ns
