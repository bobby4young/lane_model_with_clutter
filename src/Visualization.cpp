#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Graph.h"
#include "GraphNode.h"
#include "ObjectShape.h"
#include "Visualization.h"
#include "Utility.h"


namespace lane_model {

Visualization::Visualization(): graph_(NULL) {
  // colors_ = std::vector<cv::Scalar>{cv::Scalar(255,150,255), cv::Scalar(255,255,150), cv::Scalar(100,255,255), 
  //   cv::Scalar(150,150,255), cv::Scalar(150,255,150), cv::Scalar(255,150,150)};
  colors_ = std::vector<cv::Scalar>{cv::Scalar(100,100,150, 255), cv::Scalar(100,200,100, 255), cv::Scalar(100,255,255, 255), 
  cv::Scalar(125,125,41, 255), cv::Scalar(255,93,182, 255), cv::Scalar(0,165,255, 255), cv::Scalar(107,183,189, 255),cv::Scalar(153,136,119, 255), 
  cv::Scalar(255,255,150, 255), cv::Scalar(100,255,255, 255), cv::Scalar(50,150,255, 255), cv::Scalar(150,255,50, 255), cv::Scalar(255,50,150, 255)};
}

Visualization::Visualization(Graph* graph): graph_(graph) {
  colors_ = std::vector<cv::Scalar>{
    cv::Scalar(209,206,0,255), // dark turquoise	1
    cv::Scalar(139,139,0,255), // dark cyan	2
    cv::Scalar(160,158,95,255),	// cadet blue3	
    cv::Scalar(180,130,70,255),	//steel blue	4
    cv::Scalar(255,191,0,255),	//deep sky blue5	
    cv::Scalar(255,144,30,255),	//dodger blue	6
    cv::Scalar(144,128,112,255),	//dark olive green7
    cv::Scalar(235,206,135,255),	// sky blue 8
    cv::Scalar(225,105,65,255)	// royal blue	 9
  };
  // one pixel is 5 cm
  double image_width = 20 * graph_->SceneSize() + 400;
  image_size_ = cv::Size2d(image_width, image_width);
  //  cv::Size2d image_size = cv::Size2d(image_width, image_width);
  //image_ = cv::Mat::zeros(image_size, CV_8UC4);
}

cv::Mat Visualization::DrawLanes() const {
  if (graph_) {
    return DrawLanes(graph_);
  }
  else {
    utility::PrintError("No graph in visualization draw lanes");
    return cv::Mat();
  }
}

cv::Mat Visualization::DrawLanes(Graph* graph) const {
  // one pixel is 5 cm
  //double resolution = 20 * graph->SceneSize();
  //std::cout << "resolution:  " << resolution << std::endl;
  //cv::Size2d image_size = cv::Size2d(resolution, resolution);
  //cv::Mat image = cv::Mat(image_size_, CV_8UC4, cv::Scalar(255, 255, 255, 0));
  std::vector<LaneNode*> lanes = graph->GetLanes();
  cv::Mat image = DrawLanes(lanes);

  // draw center line's points
  std::vector<Point> centerline_points = graph->GetDriveDirection().GetPoints();
  std::vector<cv::Point> centerline_cv_points;
  for (auto p: centerline_points) {
    centerline_cv_points.emplace_back(20*p.cvPoint().x, 20*p.cvPoint().y);
  }
  for (auto p: centerline_cv_points) {
    cv::circle(image, p, 8, cv::Scalar(0, 255, 0, 255), CV_FILLED);    
  }
  return image;
}

cv::Mat Visualization::DrawLanes(const std::vector<LaneNode*> lanes) const {
  cv::Mat image = cv::Mat(image_size_, CV_8UC4, cv::Scalar(255, 255, 255, 0));
  int n_lanes = lanes.size();
  std::cout << "n lanes:  " << n_lanes<< " image_size " << image_size_.height << std::endl;
  for (int i = 0; i < n_lanes; i++) {
    LaneNode* lane = lanes[i];
    Polygon polygon = lane->GetPolygon();
    Polyline polyline1 = polygon.GetLeftline();
    Polyline polyline2 = polygon.GetRightline();

    std::vector<Point> points = polyline1.GetPoints();
    std::reverse(points.begin(),points.end());
    //std::cout << "n points:  " << points.size() << std::endl;
    std::vector<Point> points2 = polyline2.GetPoints();
    points.insert(points.end(), points2.begin(), points2.end());
    std::vector<cv::Point> cv_points;
    for (auto p: points) {
      cv_points.emplace_back(20*p.cvPoint().x, 20*p.cvPoint().y);
    }
    //std::cout << "n cv points:  " << cv_points.size() << std::endl;
    int n_point = static_cast<int>(cv_points.size());
    cv::Point rook_points[1][n_point];
    for (int j = 0; j < n_point; j++) {
      rook_points[0][j] = cv_points.at(j);
    }
    const cv::Point* ppt[1] = { rook_points[0] };
    int npt[] = { n_point };
    int colot_index = i % colors_.size();
    cv::fillPoly( image, ppt, npt, 1, colors_.at(colot_index), 8 );
    // for (auto point: cv_points) {
    //   cv::circle(image, point, 8, cv::Scalar(10, 10, 0, 255), 1);
    // }
    // draw line between successor and predeccessor
    LaneNode* successor = lane->GetSuccessor();
    if (successor) {
      std::vector<cv::Point> lane_center_points;
      cv::Point center_point1 = polygon.GetCenterPoint();
      cv::Point center_point2 = successor->GetPolygon().GetCenterPoint();
      lane_center_points.emplace_back(20 * center_point1.x, 20 * center_point1.y);
      lane_center_points.emplace_back(20 * center_point2.x, 20 * center_point2.y);
      cv::polylines(image, lane_center_points, false, cv::Scalar(255, 255, 255, 255), 1);
      cv::circle(image, {20 * center_point1.x, 20 * center_point1.y}, 3, cv::Scalar(255, 255, 255, 255), 2);
      cv::circle(image, {20 * center_point2.x, 20 * center_point2.y}, 3, cv::Scalar(255, 255, 255, 255), 2);
    }
  }
  return image;
}

cv::Mat Visualization::DrawLanes(const SceneNode* scene_node) const {
  std::vector<LaneNode*> lanes;
  for (auto road: scene_node->Children()) {
    for (auto lane: road->Children()) {
      lanes.push_back(dynamic_cast<LaneNode*>(lane));
    }
  }
  return DrawLanes(lanes);
}

cv::Mat Visualization::DrawBoundary(Graph* graph) const {
  std::vector<BoundaryNode*> boundaries = graph->GetBoundaries();
  return DrawBoundary(boundaries);
}

cv::Mat Visualization::DrawBoundary() const {
  if (graph_) {
    return DrawBoundary(graph_);
  }
  else {
    utility::PrintError("No graph in visualization draw lanes");
    return cv::Mat();
  }
}

cv::Mat Visualization::DrawBoundary(const std::vector<BoundaryNode*> boundaries) const {
  cv::RNG rng(12345);
  cv::Mat image = cv::Mat(image_size_, CV_8UC4, cv::Scalar(255, 255, 255, 0));
  for (auto boundary: boundaries) {
    std::vector<cv::Point> cv_points;
    Polyline polyline = boundary->GetPolyline();
    for (auto p: polyline.GetPoints()) {
      cv_points.emplace_back((20 * p.cvPoint().x), (20 * p.cvPoint().y));
    }
    cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255), 255);
    cv::polylines(image, cv_points, false, color, 4);
  }
  return image; 
}

cv::Mat Visualization::DrawLaneBoundary(Graph* graph) const {
  cv::RNG rng(12345);
  cv::Mat image = cv::Mat(image_size_, CV_8UC4, cv::Scalar(255, 255, 255, 0));
  std::vector<LaneBoundaryNode*> boundaries = graph->GetLaneBoundaries();
  for (auto boundary: boundaries) {
    std::vector<cv::Point> cv_points;
    Polyline polyline = boundary->GetPolyline();
    for (auto p: polyline.GetPoints()) {
      cv_points.emplace_back((20 * p.cvPoint().x), (20 * p.cvPoint().y));
    }
    cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255), 255);
    cv::polylines(image, cv_points, false, color, 4);
  }
  return image;
}

cv::Mat Visualization::DrawMarkings() const {
  std::vector<MarkingNode*> markings = graph_->GetMarkings();
  /// Find the rotated rectangles and ellipses for each contour
  std::vector<cv::RotatedRect> min_bounding_boxes( markings.size());
  cv::Mat image = cv::Mat(image_size_, CV_8UC4, cv::Scalar(255, 255, 255, 0));
  
  for(size_t i = 0; i < markings.size(); i++){ 
    min_bounding_boxes[i] = markings[i]->GetBox();
  }
  cv::RNG rng(12345);
  /// Draw contours + rotated rects + ellipses
  for( size_t i = 0; i< min_bounding_boxes.size(); i++ ) {
    cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255), 255);
    cv::Point2f rect_points[4]; 
    cv::Point2f scaled_rect_points[4]; 
    min_bounding_boxes[i].points( rect_points );
    for( int j = 0; j < 4; j++ ) {
      scaled_rect_points[j] =  cv::Point2f(rect_points[j].x * 20, rect_points[j].y * 20);
    }
    for( int j = 0; j < 4; j++ ) {
      cv::line( image, scaled_rect_points[j], scaled_rect_points[(j+1)%4], color, 3, 8 );
    }
  }
  Polyline polyline1 = graph_->GetPolygon().GetLeftline();
  Polyline polyline2 = graph_->GetPolygon().GetRightline();
  std::vector<cv::Point> cv_points1;
  std::vector<cv::Point> cv_points2;
  for (auto p: polyline1.GetPoints()) {
    cv_points1.emplace_back((20 * p.cvPoint().x), (20 * p.cvPoint().y));
  }
  for (auto p: polyline2.GetPoints()) {
    cv_points2.emplace_back((20 * p.cvPoint().x), (20 * p.cvPoint().y));
  }
  cv::Scalar black_color = cv::Scalar(0, 0, 0, 255);
  cv::polylines(image, cv_points1, false, black_color, 3);
  cv::polylines(image, cv_points2, false, black_color, 3);
  return image;
}

void Visualization::DrawPoints(std::vector<Point> points) {
  DrawPointsWithColor(points, 0);
}

void Visualization::DrawPointsWithColor(std::vector<Point> points, int color_index) {
  cv::Mat image = cv::Mat(image_size_, CV_8UC4, cv::Scalar(255, 255, 255, 0));
  std::vector<cv::Point> cv_points;
  if (color_index >= colors_.size() || color_index < 0) {
    std::cout << "color index error, index was " << color_index << std::endl;
    color_index = color_index % colors_.size();
    std::cout << "changed into  " << color_index << std::endl;
  }
  for (auto p: points) {
    cv_points.push_back(Point(p.x * 20, p.y * 20));
  }
  for (auto point: cv_points) {
    cv::circle(image, point, 8, colors_[color_index], CV_FILLED);
  }
  imwrite("visualized_graph.png", image);
}


} // ns/