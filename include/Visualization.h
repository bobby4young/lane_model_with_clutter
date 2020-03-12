#ifndef LANE_MODEL_VISUALIZATION_H_
#define LANE_MODEL_VISUALIZATION_H_

#include "ObjectShape.h"

namespace lane_model {

class Graph;

class Visualization {
 public:
  Visualization();
  Visualization(Graph* graph);
  cv::Mat DrawLanes() const;
  cv::Mat DrawLanes(Graph* graph) const;
  cv::Mat DrawLanes(const std::vector<LaneNode*> lanes) const;
  cv::Mat DrawLanes(const SceneNode* scene_node) const;
  cv::Mat DrawLaneBoundary(Graph* graph) const;
  cv::Mat DrawBoundary(Graph* graph) const;
  cv::Mat DrawBoundary() const;
  cv::Mat DrawBoundary(const std::vector<BoundaryNode*> boundaries) const;
  cv::Mat DrawMarkings() const;
  void DrawPoints(std::vector<Point> points);
  void DrawPointsWithColor(std::vector<Point> points, int color_index);

 private:
  Graph* graph_;
  //cv::Mat image_;
  cv::Size2d image_size_;
  std::vector<cv::Scalar> colors_;
};

} //ns
#endif