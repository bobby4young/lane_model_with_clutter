#ifndef LANE_MODEL_OBJECT_SHAPE_H_
#define LANE_MODEL_OBJECT_SHAPE_H_

#include <string>
#include <vector>
#include <utility>
#include <opencv2/core/core.hpp>

namespace lane_model {

class Box;

class Point: public cv::Point2d {
  friend class Box;
  friend class Polyline;
 public:
  Point(double x, double y);
  Point();
  Point(const Point point1, const Point point2);
  Point(const cv::Point2d cvpoint);
  cv::Point2d cvPoint() const;
  Point operator + (Point point) const;
  Point operator - (Point point) const;
  double operator * (Point point) const;
  Point operator * (double scale) const;
  Point operator * (int scale) const;
  bool operator == (const Point point) const;
  bool ApproximatelyEqual(const Point point) const;
  double Distance(const Point point) const;
  double DistanceSquared(const Point point) const;
  double ShortestDistance(const std::vector<Point> points) const;
};

typedef std::pair<Point, Point> LineSegment;

// class LineSegment {

// }

class Box : public cv::RotatedRect {
 public:
  Box();
  Box(cv::RotatedRect cv_rect);
  double ShortestDistanceToBox(const Box& box) const;
  double DistanceBetweenBoxCenter(const Box& box) const;
  void SetCenterX(std::string center_x_string);
  void SetCenterX(double center_x);
  void SetCenterY(std::string center_y_string);
  void SetCenterY(double center_y);
  void SetHeight(std::string height_string);
  void SetHeight(double height);
  void SetWidth(std::string width_string);
  void SetWidth(double width);
  void SetOrientation(std::string orientation_string);
  void SetOrientation(double orientation);
  double GetHeight() const;
  double GetWidth() const;
  double GetOrientation() const;
  //std::vector<Point> BoxSpine();
  LineSegment GetLine() const;
  void GenerateLine();
  double DistanceToPoint (Point point) const;
  void Merge(const Box& box);
  void Merge(const std::vector<Box>& boxes);
  Point Center() const;
  double CenterX() const;
  double CenterY() const;

  std::vector<double> GetRelation(const Box& other) const;
  
 private:
  bool vertices_valid;
  std::vector<Point> vertices_;
  LineSegment line_segment_;
  Point center_;
  double height_, width_;// height>= width
  double orientation_;// -90 degree < orientation <= 90 degree
};

class Polyline {
 public:
  Polyline();	
  Polyline(std::vector<Point> points);
  Polyline(LineSegment line_segment);
  int NPoints() const;
  int NLines() const;
  bool operator == (Polyline polyline) const;
  std::vector<LineSegment> GetLineSegments() const;
  std::vector<Point> GetPoints() const;
  LineSegment ClosestLineSegment(const Point point) const;
  LineSegment StraightLine() const;
  // in degree
  double AngleBetweenTwoLineSegments(LineSegment line_segment1, LineSegment line_segment2) const;
  // line 1 is point1 to point2. line2 is point2 to point3
  double AngleBetweenTwoLineSegments(Point point1, Point point2, Point point3) const;
  double AngleBetweenPolyline(const Polyline polyline) const;
  double CurveAngle() const;
  std::vector<double> CurveAngles() const;
  double CurveAngleMax() const;
  bool CheckIntersect(Polyline polyline) const;
  bool CheckIntersect(LineSegment line_segment) const;
  double DistanceTPoint(const Point point) const;
  double DistanceBetweenPolyline(const Polyline polyline) const;
  double MaxDistanceBetweenPolyline(const Polyline polyline) const;
  double MinDistanceBetweenPolyline(const Polyline polyline) const;
  /* -1: error
  	0: each point in points_ is colinear with this polyline, probabily overlapping
  	1: right
  	2: left
  	3: segments have different direction and polyline's ceter point is colinear with this polyline
  	4: segments have different direction and polyline's ceter point is on right of this polyline
  	5: segments have different direction and polyline's ceter point is on left of this polyline
  */
  int LeftOrRight(const Polyline polyline) const;
  int PointLeftOrRight(const Point point) const;
  Point GetDirection() const;
  // centerpoint is not neccessary on the polyline
  Point GetCenterPoint() const;
  // middle point is on the polyline
  Point GetMiddlePoint() const;
  double GetLength() const;
  Polyline Reverse() const;
  Polyline GetCenterLine(Polyline polyline) const;
  std::pair<Polyline, Polyline> SplitIntoTwoPolylines(const Point point) const;  
  /* insert new point into polyline
  	 1. InsertPointPushback: insert point to the end of points_
  	 2. InsertPointSort: Draw an rotated minimal bounding box around all the points. Sort points in length coordinate.
  	 3. InsertPointInsert: Try to find the closet two points in points_ and insert point between those two points.
  */
  void InsertPoint(const Point& point, int insert_option);

  Point GetPointWithPortion(double portion) const;
  std::vector<double> GetRelation(const Polyline& other) const;

 private:
  void InsertPointPushback(const Point& point);
  void InsertPointSort(const Point& point);
  void InsertPointInsert(const Point& point);
  void InsertPointAltnative(const Point& point);
  void InsertPointAfter(const Point& point, std::vector<Point>::iterator anchor_point);
 private:
  std::vector<Point> points_;
  std::vector<Point> middle_points_;
  std::vector<LineSegment> line_segments_;
  std::vector<double> line_length_;
  double length_sum_;
};

class Polygon {
 public:
  Polygon();
  Polygon(Polyline polyline);
  Polygon(Polyline polyline1, Polyline polyline2);
  Polyline GetLeftline() const;
  Polyline GetRightline() const;
  Polyline GetCenterLine() const;
  Point GetCenterPoint() const;
  double GetWidth() const;
	double GetWidthAtPoint(const Point point) const;
  bool IsValid() const;
  bool AddPolyline(Polyline polyline);
  void MergeWithPolyline(Polyline polyline);
  void MergeWithPolygon(Polygon polygon);
 private:
  Polyline left_line_;
  Polyline right_line_;
  Polyline centerline_;
  Point centerpoint_;
  double width_;
  bool complete_;
};
//typedef std::pair<Polyline, Polyline> Polygon;

bool IntersectionPointOfTwoLineSegment(LineSegment line_segment1, LineSegment line_segment2, Point& intersection_point);

}//ns
#endif //LANE_MODEL_OBJECT_SHAPE_H_