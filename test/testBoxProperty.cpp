#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ObjectShape.h"
#include "Utility.h"

using namespace lane_model;

double epsilon = 1.0e-5;
template <class T1, class T2>
void Print(std::string string,  T1 expected_value,  T2 value) {
  if (std::abs(expected_value - value) < epsilon) {
    std::cout << string << " should be " << expected_value << " and is " << value << std::endl;
  }
  else {
    std::cout << "\033[1;31mERROR: \033[0m" << string << " should be " << expected_value << " and is " << value << std::endl;
  }
}

template <class T>
void Print(std::string string,  T value) {
  std::cout << string << " is " << value << std::endl;
}

template <class T>
void PrintError(T value) {
  std::cout << "\033[1;31mERROR: \033[0m" << value << std::endl;
}

int testPointTBoxDistance() {
  // cv::size is initialized with (width, height)
  cv::RotatedRect rRect1 = cv::RotatedRect(cv::Point2f(100,100), cv::Size2f(50,100), 0);
  Box box1(rRect1);
  Point point(0, 0);
  double distance1 = box1.DistanceToPoint(point);
  Print("distance1 of box", 100, distance1);

  cv::RotatedRect rRect2 = cv::RotatedRect(cv::Point2f(50, 50 * std::sqrt(3)), cv::Size2f(300,100), -30);
  Box box2(rRect2);
  double distance2 = box2.DistanceToPoint(point);
  Print("distance2 of box", 100, distance2);

  return 0;
}

int testConnectingAngle() {
  cv::RotatedRect rRect1 = cv::RotatedRect(cv::Point2f(100,100), cv::Size2f(50,100), 0);
  Box box1(rRect1);
  double orientation1 = box1.GetOrientation();
  Print("orientation of box 1 is", 0, orientation1);

  cv::RotatedRect rRect2 = cv::RotatedRect(cv::Point2f(100,100), cv::Size2f(100, 50), 0);
  Box box2(rRect2);
  double orientation2 = box2.GetOrientation();
  Print("orientation of box 2 is", 90, orientation2);

  cv::RotatedRect rRect3 = cv::RotatedRect(cv::Point2f(500,500), cv::Size2f(100,50), 45);
  Box box3(rRect3);

  float distance_between_centers = box1.DistanceBetweenBoxCenter(box3);
  double distance_to_point = box1.DistanceToPoint(box3.Center());
  double connecting_angle = 0;
  if (distance_between_centers == 0.0) {
    connecting_angle = 0;
  }
  else {
    if (distance_to_point/distance_between_centers > 1) {
      PrintError("relation in MarkingNode has a weird angle");
    }
    else {
      connecting_angle = asin(distance_to_point/distance_between_centers) * 180 / M_PI;
    }
  }
  Print("connecting angle of two boxes", 45, connecting_angle);
  return 0;
}

int testPolylineLength(void) {
  std::cout << "testLength" << std::endl;
  cv::Mat line_image(1000, 1000, CV_8UC3, cv::Scalar(0,0,0)); 
  std::vector<Point> input_points1 = {{480, 100}, {480, 500}, {480, 800}};
  std::vector<Point> input_points2 = {{500, 10}, {560, 50},{560, 560},{560, 600},{560, 700}};
  std::vector<Point> out_points;
  Polyline line1(input_points1);
  Polyline line2(input_points2);
  Polyline line3(out_points);
  Polyline line4;
  std::cout << line1.AngleBetweenPolyline(line2) << std::endl;
  std::cout << line1.CheckIntersect(line2) << std::endl;
  
  Point new_point1(550, 900);
  // line1.InsertPoint(new_point1, 1);
  //line1.DrawPolyline(line_image, cv::Scalar(100,100,100), 2);
  
  Point new_point2(500, 500);
  line2.InsertPoint(new_point2, 4);
  //line2.DrawPolyline(line_image, cv::Scalar(255,255,255), 2);
  
  
  std::cout << "line1 length should be 700 and is " << line1.GetLength() << std::endl;
  Print("default constructor for polyline should have zero points", 0, line4.NPoints());
	//line3.DrawPolyline(line_image, cv::Scalar(0, 0,100), 1);
  //cv::polylines(line_image, out_points, false, cv::Scalar(255,255,255),1);
  imwrite("polyline.png",line_image);
  return 0;
}

void testPolylineDistance() {
  LineSegment line_segment1;
  line_segment1.first = Point(200, 200);
  line_segment1.second = Point(200, 800);
  
  LineSegment line_segment2;
  line_segment2.first = Point(800, 200);
  line_segment2.second = Point(800, 800);

  Point point(0, 0);
  Point point2(200, 200);
  double distance1 = utility::DistancePointTLineSegment(point, line_segment1);
  double distance12 = utility::DistancePointTLineSegment(point2, line_segment1);
  Print("distance between line_segment", 200* std::sqrt(2), distance1);
  Print("distance between line_segment", 0, distance12);

  std::vector<Point> input_points1 = {{350, 100}, {350, 200}, {500, 1000}};
  std::vector<Point> input_points2 = {{500, 100}, {500, 1000}};
  Polyline line1(input_points1);
  Polyline line2(input_points2);
  double distance2 = line1.DistanceBetweenPolyline(line2);
  double distance3 = line1.MaxDistanceBetweenPolyline(line2);
  double distance4 = line1.MinDistanceBetweenPolyline(line2);
  Print("distance between polyline", 75.0, distance2);
  Print("max distance between polyline", 150, distance3);
  Print("min distance between polyline", 0, distance4);
}

int testConnectingLineSegmentAngle() {
  LineSegment line_segment1;
  line_segment1.first = Point(200, 200);
  line_segment1.second = Point(200, 800);
  
  LineSegment line_segment2;
  line_segment2.first = Point(800, 200);
  line_segment2.second = Point(800, 800);
  double angle1 = utility::ConnectingAngleBwtweenLineSegment(line_segment1, line_segment2);
  Print("connecting angle line segments", 90.0, angle1);
  
  LineSegment line_segment3;
  line_segment3.first = Point(0, 500);
  line_segment3.second = Point(600, 500);
  double angle2 = utility::ConnectingAngleBwtweenLineSegment(line_segment1, line_segment3);
  Print("connecting angle line segments", 90.0, angle2);
  return 0;
}

int testPolylineAngle() {
  std::vector<Point> input_points1 = {{480, 100}, {480, 500}, {480, 800}};
  std::vector<Point> input_points2 = {{560, 50},{560, 500},{600, 500},{600, 700}};
  std::vector<Point> input_points3;
  std::vector<Point> input_points4 = {{560, 50},{560, 500}};
  std::vector<Point> input_points5 = {{560, 50},{560, 100},{560, 200},{560, 500},{600, 500},{600, 700}};
  std::vector<Point> input_points6 = {{100, 100}, {100, 500}, {100, 800}};
  std::vector<Point> input_points7 = {{600, 100}, {600, 200}};
  Polyline line1(input_points1);
  Polyline line2(input_points2);
  Polyline line3(input_points3);
  Polyline line4(input_points4);
  Polyline line5(input_points5);
  Polyline line6(input_points6);
  Polyline line7(input_points7);
  double angle1 = line1.CurveAngle();
  double angle2 = line2.CurveAngle();
  double angle3 = line3.CurveAngle();
  double angle4 = line4.CurveAngle();
  double angle5 = line5.CurveAngle();
  double angle6 = line5.CurveAngleMax();
  Print("curve angle ", 0.0, angle1);
  Print("curve angle ", 90.0, angle2);
  Print("curve angle ", 0.0, angle3);
  Print("curve angle ", 0.0, angle4);
  Print("curve angle ", 45.0, angle5);
  Print("curve angle max ", 90.0, angle6);

  double distance = line6.DistanceBetweenPolyline(line7);
  double max_distance = line6.MaxDistanceBetweenPolyline(line7);
  Print("distance ", 500.0, distance);
  Print("max_distance ", 500.0, max_distance);

  double angle7 = utility::AngleBetweenTwoLineSegments(line6.StraightLine(), line7.StraightLine());
  Print("angle7 ", 0.0, angle7);

  Point point(100, 600);
  int side = line1.PointLeftOrRight(point);
  Print("side ", 2, side);
  
  Point centerpoint = line6.GetMiddlePoint();
  std::string point_coordinate = std::to_string(centerpoint.x) + " " + std::to_string(centerpoint.y);    	
	Print("GetMiddlePoint", point_coordinate);	
  return 0;
}

bool testPointInPolyline() {
  std::vector<Point> input_points1 = {{480, 100}, {480, 500}, {480, 800}};
  Polyline line1(input_points1);
  Point middle_point = line1.GetPointWithPortion(0.5);
  Print("middle_point x ", 480, middle_point.x);
  Print("middle_point y ", 450, middle_point.y);
}

int main(void) {
  testPointTBoxDistance();
  testConnectingAngle();
  testPolylineLength();
  testPolylineDistance();
  testConnectingLineSegmentAngle();
  testPolylineAngle();
  testPointInPolyline();
  return 0;
}
