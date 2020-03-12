#include <assert.h>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>
#include <set>
#include <string.h>
#include <stdlib.h>     /* srand, rand */
#include "Edge.h"
#include "Grammar.h"
#include "Graph.h"
#include "GraphNode.h"
#include "Label.h"
#include "Rule.h"
#include "Utility.h"

namespace lane_model{

GraphNode::GraphNode(Graph* pGraph): graph_(pGraph),
  label_(NULL),
  graph_index_(-1),
  parent_(NULL),
  children_(0, NULL) {
	if(pGraph) {
    pGraph->InsertNode(this);
  }
}

GraphNode::GraphNode(Graph* pGraph, int index): graph_(pGraph),
  label_(NULL),
  graph_index_(index),
  parent_(NULL),
  children_(0, NULL) {
	if(pGraph) pGraph->InsertNode(this);
}

GraphNode::GraphNode(const GraphNode& other): graph_(NULL),
  label_(other.label_),
  rule_(other.rule_),
  graph_index_(other.graph_index_) {}

GraphNode& GraphNode::operator = (const GraphNode& other) {
  // Don't allow assigment inside one graph. Avoid duplicating nodes inside a graph.
  assert(graph_ != other.graph_);
  label_ = other.label_;
  rule_ = other.rule_;
  graph_index_ = other.graph_index_;
  return *this;
}

void GraphNode::InsertChild(GraphNode* pNode) {
	// Check scene
	assert(pNode->graph_ == graph_);
	assert(pNode->graph_index_ >= 0);
	
	// Insert node into children
	pNode->parent_ = this;
	children_.push_back(pNode);
}

Graph* GraphNode::MyGraph(void) const {
  return graph_;
}

int GraphNode::NChildren(void) const {
  return children_.size();
}

GraphNode* GraphNode::Child(int i) const {
  assert( i >= 0 && i < children_.size());
  return children_[i];
}

GraphNode* GraphNode::ParentNode() const {
  return parent_;
}

std::vector<GraphNode*> GraphNode::Children() const {
  return children_;
}

bool GraphNode::IsTerminal() const {
  if (NChildren() == 0) return true;
  else return false;
}

void GraphNode::SetAssociatedLabel(Label* pLabel) {
  assert(!pLabel || pLabel->GetGrammar() == MyGraph()->GetGrammar());
	label_ = pLabel;
}

void GraphNode::SetAssociatedRule(Rule* pRule) {
  assert(!pRule || pRule->GetGrammar() == MyGraph()->GetGrammar());
  rule_ = pRule;
}

Label* GraphNode::AssociatedLabel(void) const {
  return label_;
}

Rule* GraphNode::AssociatedRule() const {
  return rule_;
}

int GraphNode::GraphIndex() const {
  return graph_index_;
}

std::string GraphNode::ToString() const {
  return label_->BasicCategoryName() + std::to_string(graph_index_);
}

bool GraphNode::operator == (GraphNode* pNode) const {
  return graph_index_ == pNode->graph_index_;
}

bool GraphNode::AddSibling(GraphNode* pNode) {
  if (this == pNode) return false;  
  Label* label = pNode->AssociatedLabel();
  LabelEdge* pLabel_edge = graph_->GetGrammar()->GetLabelEdgeByInstance(rule_, label_, label);

  if (pLabel_edge->IsEdge(this, pNode)) {
    siblings_.emplace(pNode);
    pNode->siblings_.emplace(this);
    return true;
  }
  else {
    return false;
  }
}

bool GraphNode::IsSibling(GraphNode* pNode) const {
  // node itself is not its own sibling
  if (siblings_.find(pNode) == siblings_.end() || pNode == this) {
    return false;  
  }
  else {
    return true;
  }
}
bool GraphNode::HasSiblingInChildren(GraphNode* pNode) const {
  for (auto child: children_) {
    if (pNode->IsSibling(child)) {
      return true;
    }
  }
  return false;
}

bool GraphNode::HasSameChild(GraphNode* pNode) const {
  if (pNode->NChildren() == 0 || NChildren() == 0) {
    return false;
  }
  for (int i = 0; i < pNode->NChildren(); i++) {
    if (std::find(children_.begin(), children_.end(), pNode->Child(i)) != children_.end()) {
      return true;
    }
  }
  return false;
}

void GraphNode::PrintSibling() const {
  std::cout << "siblings in " << ToString() << " : ";
  for (auto sibling: siblings_) {
    std::cout << " " << sibling->ToString();
  }
  std::cout << std::endl;
}

MarkingNode::MarkingNode(Graph* pGraph, int index): GraphNode(pGraph, index) {
  pGraph->InsertMarkingNode(this);
}

MarkingNode::MarkingNode(Graph* pGraph, int index, Box bounding_box): GraphNode(pGraph, index),
  bounding_box_(bounding_box){
  pGraph->InsertMarkingNode(this);
}

MarkingNode::MarkingNode(const MarkingNode& other): GraphNode(other),
  bounding_box_(other.bounding_box_) {}

MarkingNode& MarkingNode::operator = (const MarkingNode& other) {
  GraphNode::operator=(other);
  bounding_box_ = other.bounding_box_;
  return *this;
}

void MarkingNode::SetBox(cv::RotatedRect cv_rect) {
  bounding_box_ = Box(cv_rect);
}

LineSegment MarkingNode::GetLineSegment() const {
  return bounding_box_.GetLine();
}

std::vector<double> MarkingNode::Value() const {
  double distance_epsilon = (rand() % 10 - 5) / 1.0e4;
  std::vector<double> value;
  value.push_back(bounding_box_.GetHeight());
  value.push_back(bounding_box_.GetWidth() + distance_epsilon);
  //value.push_back(bounding_box_.GetHeight() / bounding_box_.GetWidth());
  //value.push_back(bounding_box_.GetOrientation());
  return value;
}

std::vector<double> MarkingNode::Relation(const GraphNode* pNode) const {
  const MarkingNode* pMarking = dynamic_cast<const MarkingNode*>(pNode);
  Box other_bounding_box = pMarking->bounding_box_;
  std::vector<double> relation(4, 0.0);
  // distance from point to spine line
  relation[0] = bounding_box_.DistanceToPoint(other_bounding_box.Center());
  //std::cout << " DistanceTPoint: " << distance[0] << "  "; 
  //distance[1] = bounding_box_.DistanceBetweenBoxCenter(descriptor->bounding_box_);
  // alignment
  //relation[1] = std::abs(bounding_box_.GetOrientation() - other_bounding_box.GetOrientation());
  // closest distance between bounding box
  relation[1] = bounding_box_.ShortestDistanceToBox(other_bounding_box);
  //length ratio
  double ratio = (double)(bounding_box_.GetHeight() / other_bounding_box.GetHeight());
  assert( ratio > 0 );
  relation[2] = std::max(ratio, 1 / ratio);
  //relation[2] = ratio;
  // //angle between orientation and line connecting two centers
  // double orientation = bounding_box_.GetOrientation();
  // double dx = other_bounding_box.CenterX() - bounding_box_.CenterX();
  // double dy = other_bounding_box.CenterY() - bounding_box_.CenterY();
  // double connecting_angle = atan2(dy, dx) * 180 / M_PI;
  // connecting_angle = std::abs(connecting_angle - orientation - 90);
  // if (connecting_angle >= 90) {
  //   connecting_angle = 180 - connecting_angle;
  // }
  // relation[3] = std::abs(connecting_angle);
/////////////////////////////
  float distance_between_centers = bounding_box_.DistanceBetweenBoxCenter(other_bounding_box);
  double connecting_angle = 0;
  if (distance_between_centers == 0.0) {
    connecting_angle = 0;
  }
  else {
    if (relation[0]/distance_between_centers > 1) {
      std::cout << "relation in MarkingNode has a weird angle"<< std::endl;
      //exit(1);
    }
    else {
      connecting_angle = asin(relation[0]/distance_between_centers) * 180 / M_PI;
    }
  }
  
  relation[3] = std::abs(connecting_angle);
/////////////////////////////
  //std::cout << " alignment: " << relation[1] << " " << "connecting_angle: " << connecting_angle <<" ";
  //float distance_between_centers = bounding_box_.DistanceBetweenBoxCenter(other_bounding_box);
  //relation[4] = distance_between_centers / bounding_box_.GetHeight();
  //relation[4] = bounding_box_.DistanceBetweenBoxCenter(other_bounding_box); 
  return relation;
}

bool MarkingNode::AddSibling(GraphNode* pNode) {
  if (this == pNode) return false;
  MarkingNode* marking_node = dynamic_cast<MarkingNode*>(pNode);
  Label* label = marking_node->AssociatedLabel();
  LabelEdge* pLabel_edge = graph_->GetGrammar()->GetLabelEdgeByInstance(rule_, label_, label);
  if (pLabel_edge->IsEdge(this, pNode)) {
    siblings_.emplace(pNode);
    GraphNode* this_node = dynamic_cast<GraphNode*>(this);
    marking_node->siblings_.emplace(this_node);
    return true;
  }
  else{
    return false;
  }
}

int MarkingNode::Position(GraphNode* pNode) const {
  return 0;
}

GraphNode* MarkingNode::ClosestGraphNode(std::vector<GraphNode*> siblings) const {
  std::vector<float> distance_array;
  for (int i = 0; i< (int)siblings.size(); i++) {
    if ( this == siblings[i]) {
      distance_array.push_back(1.0e10);
      continue;
    }
    MarkingNode* marking_node = dynamic_cast<MarkingNode*>(siblings[i]); 
    Box box2 = marking_node->bounding_box_;
    distance_array.push_back(bounding_box_.DistanceBetweenBoxCenter(box2));
  }
  std::vector<float>::iterator lowest = std::min_element(std::begin(distance_array), std::end(distance_array));
  int index = std::distance(std::begin(distance_array), lowest);
  return siblings[index];
}

Box MarkingNode::GetBox() const {
  return bounding_box_;
}

BoundaryNode::BoundaryNode(Graph* pGraph, int index): GraphNode(pGraph, index) {
  pGraph->InsertBoundaryNode(this);
}

BoundaryNode::BoundaryNode(Graph* pGraph, int index, Polyline polyline) : GraphNode(pGraph, index), polyline_(polyline) {
  pGraph->InsertBoundaryNode(this);
}

BoundaryNode::BoundaryNode(const BoundaryNode& other): GraphNode(other),
  polyline_(other.polyline_) {}

BoundaryNode& BoundaryNode::operator = (const BoundaryNode& other) {
  GraphNode::operator=(other);
  polyline_ = other.polyline_;
  return *this;
}

Polyline BoundaryNode::GetPolyline() const {
  return polyline_;
}

void BoundaryNode::AddMarking(GraphNode* pNode) {
  MarkingNode* marking_node = dynamic_cast<MarkingNode*>(pNode);  
  LineSegment line_segment = marking_node->GetLineSegment();
  if (polyline_.NPoints() == 0) {
    // polyline not initialized yet
    polyline_ = Polyline(line_segment);
  }
  else {
    // insert points into polyline_
    polyline_.InsertPoint(line_segment.first, 3);
    polyline_.InsertPoint(line_segment.second, 3);
  }
  InsertChild(pNode);
}

int BoundaryNode::Position(GraphNode* pNode) const {
  BoundaryNode* boundary_node = dynamic_cast<BoundaryNode*>(pNode);
  int position = polyline_.LeftOrRight(boundary_node->GetPolyline());
  return position;
}

std::vector<double> BoundaryNode::Value() const {
  // std::vector<double> value(6, 0.0);
  // double length = polyline_.GetLength();
  // value.at(0) = length;
  // //value.push_back(length / driveing_direction.GetLength());
  // if (NChildren() < 2) {
  //   double curve_angle = polyline_.CurveAngle();
  //   value.at(1) = 1.0;
  //   value.at(2) = 0;
  //   value.at(3) = 0;
  //   value.at(4) = curve_angle;
  //   value.at(5) = curve_angle;
  //   return value;
  // }
  // double markings_length = 0.0;
  // for (auto marking: children_) {
  //   markings_length += utility::LineSegmentLength(marking->GetLineSegment());
  // }
  // value.at(1) = markings_length / length;
  
  // std::vector<double> markings_length_array;
  // std::vector<double> visual_length_array;
  // std::vector<LineSegment> line_segments = polyline_.GetLineSegments();
  // bool is_marking = true;
  // for (std::vector<LineSegment>::iterator it = line_segments.begin(); it != line_segments.end(); it++) {
  //   double length = utility::LineSegmentLength(*it);
  //   if (is_marking) {
  //     markings_length_array.push_back(length);
  //     is_marking = false;
  //   }
  //   else {
  //     visual_length_array.push_back(length);
  //     is_marking = true;
  //   }
  // }
  // double marking_sum = std::accumulate(markings_length_array.begin(), markings_length_array.end(), 0.0);
  // double marking_mean = marking_sum / markings_length_array.size();
  // double marking_squared_sum = std::inner_product(markings_length_array.begin(), markings_length_array.end(), markings_length_array.begin(), 0.0);
  // double marking_standard_dev = std::sqrt(marking_squared_sum / markings_length_array.size() - marking_mean * marking_mean);

  // double visual_sum = std::accumulate(visual_length_array.begin(), visual_length_array.end(), 0.0);
  // double visual_mean = visual_sum / visual_length_array.size();
  // double visual_squared_sum = std::inner_product(visual_length_array.begin(), visual_length_array.end(), visual_length_array.begin(), 0.0);
  // double visual_standard_dev = std::sqrt(visual_squared_sum / visual_length_array.size() - visual_mean * visual_mean);

  // value.at(2) = marking_standard_dev; 
  // value.at(3) = visual_standard_dev;
  // // value.at(4) = marking_mean / visual_mean;
  // // value.at(5) = 0;
  // // std::cout << "value in boundary " << std::endl;
  // // for (auto v: value) {
  // //   std::cout << " " << v;
  // // }

  // Polyline driveing_direction = graph_->GetDriveDirection();
  // double angle_to_driving_direction = polyline_.AngleBetweenPolyline(driveing_direction);
  // //std::cout << "angle_to_driving_direction is " << angle_to_driving_direction << std::endl;
  // double curve_angle = polyline_.CurveAngle();
  // value.at(4) = curve_angle;
  // double max_curve_angle = polyline_.CurveAngleMax();
  // value.at(5) = max_curve_angle;
  std::vector<double> angles = polyline_.CurveAngles();
  int n_angles = angles.size();
  double mean = std::accumulate(angles.begin(), angles.end(), 0.0) / n_angles;
  double variance = 0.0;
  for(int i = 0; i < n_angles; i ++) {
    variance += (angles[i] - mean) * (angles[i] - mean);
  }
  variance /= n_angles;

  Polyline driveing_direction = graph_->GetDriveDirection();
  double angle_to_driving_direction = polyline_.AngleBetweenPolyline(driveing_direction);

  std::vector<double> value;
  double length = polyline_.GetLength();
  value.push_back(length);
  double curve_angle = polyline_.CurveAngle();
  value.push_back(curve_angle);
  double max_curve_angle = polyline_.CurveAngleMax();
  value.push_back(max_curve_angle);
  value.push_back(variance);
  value.push_back(angle_to_driving_direction);
  return value;
}

std::vector<double> BoundaryNode::Relation(const GraphNode* pNode) const {
  const BoundaryNode* pBoundary = dynamic_cast<const BoundaryNode*>(pNode);
  Polyline other_polyline = pBoundary->polyline_;
  std::vector<double> relation(5, 0.0);
  relation[0] = polyline_.AngleBetweenPolyline(other_polyline);
  relation[1] = polyline_.DistanceBetweenPolyline(other_polyline);
  relation[2] = polyline_.GetLength() / other_polyline.GetLength();
  relation[3] = polyline_.MaxDistanceBetweenPolyline(other_polyline);
  relation[4] = polyline_.MinDistanceBetweenPolyline(other_polyline);
  return relation;
}

bool BoundaryNode::AddSibling(GraphNode* pNode) {
  if (this == pNode) return false;
  BoundaryNode* sibling = dynamic_cast<BoundaryNode*>(pNode);
  // if pNode has this node's child, then they can't be siblings
  if (HasSameChild(pNode)) {
    return false;
  }
  // if the likelihood is below threshold, they are not siblings
  Label* label = pNode->AssociatedLabel();
  Grammar* grammar = graph_->GetGrammar();
  LabelEdge* pLabel_edge = grammar->GetLabelEdgeByInstance(rule_, label, label_);
  if (pLabel_edge->IsEdge(this, pNode)) {
    siblings_.emplace(pNode);
    sibling->siblings_.emplace(this);
    return true;
  }
  else {
    return false;
  }
}

LineSegment BoundaryNode::GetLineSegment() const {
  return polyline_.StraightLine();
}

GraphNode* BoundaryNode::ClosestGraphNode(std::vector<GraphNode*> siblings) const {
  std::vector<float> distance_array;
  for (int i = 0; i < siblings.size(); i++) {
    if ( this == siblings[i]) {
      distance_array.push_back(1.0e10);
      continue;
    }
    BoundaryNode* boundary_node = dynamic_cast<BoundaryNode*>(siblings[i]); 
    Polyline other_polyline = boundary_node->polyline_;
    distance_array.push_back(polyline_.DistanceBetweenPolyline(other_polyline));
  }
  std::vector<float>::iterator lowest = std::min_element(std::begin(distance_array), std::end(distance_array));
  int index = std::distance(std::begin(distance_array), lowest);
  return siblings[index];
}

std::pair<Polyline, Polyline> BoundaryNode::Split(Point point) {
  return polyline_.SplitIntoTwoPolylines(point);
}

bool BoundaryNode::IsIntersect(const LineSegment line_segment) const {
	return polyline_.CheckIntersect(line_segment);
}

bool BoundaryNode::IsIntersect(const BoundaryNode* other_boundary) const {
	return polyline_.CheckIntersect(other_boundary->GetPolyline());
}

bool BoundaryNode::IsSubset(BoundaryNode* boundary_node) const {
  if (this == boundary_node) {
    return false;
  }
  std::vector<GraphNode*> other_children = boundary_node->Children();
  for (auto c: other_children) {
    if (std::find(children_.begin(), children_.end(), c) == children_.end()) {
      return false;
    }
  }
  return true;
}

LaneBoundaryNode::LaneBoundaryNode(Graph* pGraph, int index): GraphNode(pGraph, index) {
  pGraph->InsertLaneBoundaryNode(this);
}

LaneBoundaryNode::LaneBoundaryNode(Graph* pGraph, int index, Polyline polyline) : GraphNode(pGraph, index), polyline_(polyline) {
  pGraph->InsertLaneBoundaryNode(this);
}

LaneBoundaryNode::LaneBoundaryNode(const LaneBoundaryNode& other): GraphNode(other),
  polyline_(other.polyline_) {}

LaneBoundaryNode& LaneBoundaryNode::operator = (const LaneBoundaryNode& other) {
  GraphNode::operator=(other);
  polyline_ = other.polyline_;
  return *this;
}

Polyline LaneBoundaryNode::GetPolyline() const {
  return polyline_;
}

int LaneBoundaryNode::Position(GraphNode* pNode) const {
  const LaneBoundaryNode* boundary_node = dynamic_cast<const LaneBoundaryNode*>(pNode);
  Polyline polyline = boundary_node->GetPolyline();
  int position = polyline_.PointLeftOrRight(polyline.GetCenterPoint());
  return position;
}

std::vector<double> LaneBoundaryNode::Value() const {
  std::vector<double> value(4, 0.0);
  Polyline driveing_direction = graph_->GetDriveDirection();
  double angle_to_driving_direction = polyline_.AngleBetweenPolyline(driveing_direction);
  value.at(0) = angle_to_driving_direction;
  double length = polyline_.GetLength();
  value.at(1) = length;
  double curve_angle = polyline_.CurveAngle();
  value.at(2) = curve_angle;
  double max_curve_angle = polyline_.CurveAngleMax();
  value.at(3) = max_curve_angle;
  return value;
}

std::vector<double> LaneBoundaryNode::Relation(const GraphNode* pNode) const {
  const LaneBoundaryNode* pBoundary = dynamic_cast<const LaneBoundaryNode*>(pNode);
  Polyline other_polyline = pBoundary->polyline_;
  Point centerpoint1 = polyline_.GetCenterPoint();
  Point centerpoint2 = other_polyline.GetCenterPoint();
  Polyline centerline = polyline_.GetCenterLine(other_polyline);
  LineSegment line_segment{centerpoint1, centerpoint2};
  double middle_point_angle = utility::AngleBetweenTwoLineSegments(line_segment, centerline.StraightLine());
  //utility::PrintError("middle_point_angle", middle_point_angle);
  std::vector<double> relation(5, 0.0);
  relation[0] = polyline_.AngleBetweenPolyline(other_polyline);
  relation[1] = polyline_.DistanceBetweenPolyline(other_polyline);
  relation[2] = polyline_.GetLength() / other_polyline.GetLength();
  relation[3] = polyline_.MaxDistanceBetweenPolyline(other_polyline);
  relation[4] = polyline_.MinDistanceBetweenPolyline(other_polyline);
  //relation[5] = middle_point_angle;
  return relation;
}

bool LaneBoundaryNode::AddSibling(GraphNode* pNode) {
  if (this == pNode) return false;
  if (siblings_.find(pNode) != siblings_.end()) {
    return true;
  }
  LaneBoundaryNode* sibling = dynamic_cast<LaneBoundaryNode*>(pNode);
  // if pNode has this node's child, then they can't be siblings
  if (HasSameChild(pNode)) {
    return false;
  }
  // if the likelihood is below threshold, they are not siblings
  Label* label = pNode->AssociatedLabel();
  std::cout << label_->BasicCategoryName() << " " << label->BasicCategoryName() << std::endl;
  Grammar* grammar = graph_->GetGrammar();
  LabelEdge* pLabel_edge = grammar->GetLabelEdgeByInstance(rule_, label, label_);
  if (pLabel_edge->IsEdge(this, pNode)) {
    siblings_.emplace(pNode);
    sibling->siblings_.emplace(this);
    return true;
  }
  else{
    return false;
  }
}

LineSegment LaneBoundaryNode::GetLineSegment() const {
  return polyline_.StraightLine();
}

GraphNode* LaneBoundaryNode::ClosestGraphNode(std::vector<GraphNode*> siblings) const {
  std::vector<float> distance_array;
  for (int i = 0; i < siblings.size(); i++) {
    if ( this == siblings[i]) {
      distance_array.push_back(1.0e10);
      continue;
    }
    LaneBoundaryNode* boundary_node = dynamic_cast<LaneBoundaryNode*>(siblings[i]); 
    Polyline other_polyline = boundary_node->polyline_;
    distance_array.push_back(polyline_.DistanceBetweenPolyline(other_polyline));
  }
  std::vector<float>::iterator lowest = std::min_element(std::begin(distance_array), std::end(distance_array));
  int index = std::distance(std::begin(distance_array), lowest);
  return siblings[index];
}

std::pair<Polyline, Polyline> LaneBoundaryNode::Split(Point point) {
  return polyline_.SplitIntoTwoPolylines(point);
}

bool LaneBoundaryNode::IsIntersect(LineSegment line_segment) const {
	return polyline_.CheckIntersect(line_segment);
}

void LaneBoundaryNode::AddSuccessor(LaneBoundaryNode* lane_boundary_node) {
  successor_ = lane_boundary_node;
  lane_boundary_node->successor_ = this;
}

bool LaneBoundaryNode::IsSuccessor(LaneBoundaryNode* lane_boundary_node) const {
 return successor_ == lane_boundary_node;
}

LaneNode::LaneNode(Graph* pGraph, int index): GraphNode(pGraph, index), successor_(NULL) {
  pGraph->InsertLaneNode(this);
}

LaneNode::LaneNode(Graph* pGraph, int index, Polygon polygon) : GraphNode(pGraph, index), polygon_(polygon) {
  pGraph->InsertLaneNode(this);
}

LaneNode::LaneNode(const LaneNode& other): GraphNode(other),
  polygon_(other.polygon_) {}

LaneNode& LaneNode::operator = (const LaneNode& other) {
  GraphNode::operator=(other);
  polygon_ = other.polygon_;
  return *this;
}

void LaneNode::AddBoundary(GraphNode* pNode) {
  LaneBoundaryNode* boundary_node = dynamic_cast<LaneBoundaryNode*>(pNode);
  Polyline polyline =  boundary_node->GetPolyline();
  LineSegment line_segment = polyline.StraightLine();
  if (NChildren() == 0) {
    InsertChild(pNode);
    polygon_ = Polygon(polyline);
  }
  else if (NChildren() == 1) {
    polygon_.AddPolyline(polyline);
    InsertChild(pNode);
  }
  else {
    utility::PrintError("LaneNode is trying to add more boundary to a lane which already has two boundaries");
  }
}

std::vector<double> LaneNode::Value() const {
  double length_ratio = polygon_.GetLeftline().GetLength() / polygon_.GetRightline().GetLength();
  if (length_ratio < 1) {
    length_ratio = 1 / length_ratio; 
  }
  Polyline centerline = polygon_.GetCenterLine();
  Point point1 = polygon_.GetLeftline().GetMiddlePoint();
  Point point2 = polygon_.GetRightline().GetMiddlePoint();
  double cross_angle = utility::AngleBetweenTwoLineSegments(centerline.StraightLine(), LineSegment{point1, point2});
  //utility::PrintWarning("angle", angle);
  std::vector<double> value;
  //value.push_back(polygon_.GetWidth());
  //value.push_back(polygon_.GetWidthAtPoint(polygon_.GetCenterPoint()));
  value.push_back(polygon_.GetLeftline().AngleBetweenPolyline(polygon_.GetRightline()));
  //value.push_back(angle);
  value.push_back(polygon_.GetLeftline().DistanceBetweenPolyline(polygon_.GetRightline()));
  value.push_back(polygon_.GetLeftline().MinDistanceBetweenPolyline(polygon_.GetRightline()));
  value.push_back(polygon_.GetLeftline().MaxDistanceBetweenPolyline(polygon_.GetRightline()));
  //value.push_back(polygon_.GetLeftline().GetLength() / polygon_.GetRightline().GetLength());
  value.push_back(length_ratio);
  return value;
}

std::vector<double> LaneNode::Relation(const GraphNode* pNode) const {
  const LaneNode* pLane = dynamic_cast<const LaneNode*>(pNode);
  Polygon other_polygon = pLane->polygon_;
  std::vector<double> relation(5, 0.0);
  Polyline centerline1 = polygon_.GetCenterLine();
  Polyline centerline2 = other_polygon.GetCenterLine();
  relation[0] = centerline1.AngleBetweenPolyline(centerline2);
  relation[1] = centerline1.DistanceBetweenPolyline(centerline2);
  relation[2] = centerline1.GetLength() / centerline2.GetLength();
  relation[3] = centerline1.MaxDistanceBetweenPolyline(centerline2);
  relation[4] = centerline1.MinDistanceBetweenPolyline(centerline2);
  return relation;
}

/*
  AddSibling consider geometry relation, a.k.a edge. Override AddSibling
*/
bool LaneNode::AddSibling(GraphNode* pNode) {
  if (this == pNode) return false;
  LaneNode* sibling = dynamic_cast<LaneNode*>(pNode);
  if (NChildren() != 2 || pNode->NChildren() != 2) {
    std::cout << "there is a lane node, which doesn't have two boundaries" << std::endl;
    return false;
  }
  GraphNode* boundary1 = Child(0);
  GraphNode* boundary2 = Child(1);
  GraphNode* boundary3 = pNode->Child(0);
  GraphNode* boundary4 = pNode->Child(1);
  
  if (pNode->HasSiblingInChildren(boundary1) && pNode->HasSiblingInChildren(boundary2) ) {
    Label* label = sibling->AssociatedLabel();
    LabelEdge* pLabel_edge = graph_->GetGrammar()->GetLabelEdgeByInstance(rule_, label_, label);
    // if the likelihood is below threshold, they are not siblings
    if (pLabel_edge->IsEdge(this, pNode)) {
      siblings_.emplace(pNode);
      sibling->siblings_.emplace(this);
      std::cout << ToString() <<" and " << pNode->ToString() << " are siblings" << std::endl;
      return true;
    }
  }
  else {
    return false;
  }
}

int LaneNode::Position(GraphNode* pNode) const {
  const LaneNode* pLane = dynamic_cast<const LaneNode*>(pNode);
  Polyline polyline = pLane->GetCenterline();
  int position = centerline_.LeftOrRight(polyline);
  return position;
}

LineSegment LaneNode::GetLineSegment() const {
  return centerline_.StraightLine();
}

GraphNode* LaneNode::ClosestGraphNode(std::vector<GraphNode*> siblings) const {
  std::vector<float> distance_array;
  for (int i = 0; i < siblings.size(); i++) {
    if ( this == siblings[i]) {
      distance_array.push_back(1.0e10);
      continue;
    }
    LaneNode* lane_node = dynamic_cast<LaneNode*>(siblings[i]); 
    Point other_point = lane_node->polygon_.GetCenterPoint();
    distance_array.push_back(polygon_.GetCenterPoint().Distance(other_point));
  }
  std::vector<float>::iterator lowest = std::min_element(std::begin(distance_array), std::end(distance_array));
  int index = std::distance(std::begin(distance_array), lowest);
  return siblings[index];
}

Polygon LaneNode::GetPolygon() const {
  return polygon_;
}

Polyline LaneNode::GetCenterline() const {
  return centerline_;
}

Point LaneNode::GetCenterPoint() const {
  return polygon_.GetCenterPoint();
}

LaneNode* LaneNode::GetSuccessor() const {
  return successor_;
}

void LaneNode::AddSuccessor(LaneNode* lane) {
  if (!successor_) {
    successor_ = lane;
    lane->successor_ = this;
  }
}

bool LaneNode::IsSuccessor(LaneNode* lane_node) const {
  return successor_ == lane_node;
}

void LaneNode::CheckSuccessor(LaneNode* lane_node) {
  /* 
    Don't need to check those two lanes belong to different roads. 
    The combination parsing could have assigned this lane to different roads, so the parent might not be included in this graph.
  */
  Polyline left_line1 = polygon_.GetLeftline();
  Polyline right_line1 = polygon_.GetRightline();
  Polyline left_line2 = lane_node->GetPolygon().GetLeftline();
  Polyline right_line2 = lane_node->GetPolygon().GetRightline();
  if (left_line1.MinDistanceBetweenPolyline(left_line2) < 3.0 && right_line1.MinDistanceBetweenPolyline(right_line2) < 3.0) {
    AddSuccessor(lane_node);
  }
}

/*
  CheckAddSibling doesn't consider geometry relation. If two lanes share one lane boundary, add them as siblings
*/
bool LaneNode::CheckAddSibling(LaneNode* lane_node) {
  if (this == lane_node) return false;
  if (IsSibling(lane_node)){
    siblings_.insert(lane_node->siblings_.begin(), lane_node->siblings_.end());
    lane_node->siblings_.insert(siblings_.begin(), siblings_.end());
    //std::cout << "about to return sibling " << ToString()<< " with " << lane_node->ToString() << std::endl;
    return true;
  }
  GraphNode* lane_boundary1 = Child(0);
  GraphNode* lane_boundary2 = Child(1);
  GraphNode* lane_boundary3 = lane_node->Child(0);
  GraphNode* lane_boundary4 = lane_node->Child(1);
  if (lane_boundary1 == lane_boundary3 || lane_boundary1 == lane_boundary4 || lane_boundary2 == lane_boundary3 || lane_boundary2 == lane_boundary4) {
    siblings_.insert(lane_node->siblings_.begin(), lane_node->siblings_.end());
    lane_node->siblings_.insert(siblings_.begin(), siblings_.end());
    siblings_.emplace(lane_node);
    lane_node->siblings_.emplace(this);
    std::cout << "adding sibling " << ToString()<< " with " << lane_node->ToString() << std::endl;
    return true;
  }
  else {
    return false;
  }
}
  
RoadNode::RoadNode(Graph* pGraph, int index): GraphNode(pGraph, index) {
  pGraph->InsertRoadNode(this);
}

RoadNode::RoadNode(Graph* pGraph, int index, Polygon polygon) : GraphNode(pGraph, index), polygon_(polygon) {
  pGraph->InsertRoadNode(this);
}

RoadNode::RoadNode(const RoadNode& other): GraphNode(other),
  polygon_(other.polygon_) {}

RoadNode& RoadNode::operator = (const RoadNode& other) {
  GraphNode::operator=(other);
  polygon_ = other.polygon_;
  return *this;
}

void RoadNode::AddLane(GraphNode* pNode) {
  LaneNode* lane_node = dynamic_cast<LaneNode*>(pNode);
  Polygon polygon = lane_node->GetPolygon();
  if (NChildren() == 0) {
    polygon_ = polygon;
  }
  else {
    polygon_.MergeWithPolygon(polygon);
  }
  centerline_ = polygon_.GetCenterLine();
  center_point_ = polygon_.GetCenterPoint();
  InsertChild(pNode);
}

std::vector<double> RoadNode::Value() const {
  std::vector<double> lane_width;
  lane_width.reserve(children_.size());
  for (auto child: children_) {
    const LaneNode* lane = dynamic_cast<const LaneNode*>(child);
    lane_width.push_back(lane->GetPolygon().GetWidth());
  }
  std::vector<double> value;
  value.push_back(*max_element(lane_width.begin(), lane_width.end()));
  value.push_back(*min_element(lane_width.begin(), lane_width.end()));
  value.push_back(polygon_.GetWidth() / NChildren());
  double distance_between_boundaries = graph_->GetGraphWidthAtPoint(center_point_);
  double width_ratio = distance_between_boundaries / polygon_.GetWidth();
  if (width_ratio < 1) {
    width_ratio = 1 / width_ratio;
  }
  value.push_back(width_ratio);
  /*
  avoid overlapping
  */
  double lane_width_sum = std::accumulate(lane_width.begin(), lane_width.end(), 0.0);
  double ratio2 = lane_width_sum / polygon_.GetWidth();
  value.push_back(ratio2);
  return value;
}

std::vector<double> RoadNode::Relation(const GraphNode* pNode) const {
  std::vector<double> relation(3, 0.0);
  const RoadNode* road_node = dynamic_cast<const RoadNode*>(pNode);
  Polygon other_polygon = road_node->GetPolygon();
  double width_ratio = polygon_.GetWidth() / other_polygon.GetWidth();
  if (width_ratio < 1) {
    width_ratio = 1 / width_ratio;
  }
  relation[0] = width_ratio;
  Polyline other_centerline = other_polygon.GetCenterLine();
  relation[1] = centerline_.AngleBetweenTwoLineSegments(centerline_.StraightLine(), other_centerline.StraightLine());
  double distance_epsilon = (rand() % 10 - 5) / 1.0e3;
  relation[2] = utility::DistanceBetweenTwoLineSegments(centerline_.StraightLine(), other_centerline.StraightLine()) + distance_epsilon;
  return relation;
}

bool RoadNode::AddSibling(GraphNode* pNode) {
  return false;
}

int RoadNode::Position(GraphNode* pNode) const {
  return 0;
}

LineSegment RoadNode::GetLineSegment() const {
  Polyline polyline = polygon_.GetCenterLine();
  return polyline.StraightLine();
}

GraphNode* RoadNode::ClosestGraphNode(std::vector<GraphNode*> siblings) const {
  return siblings.front();
}

Polygon RoadNode::GetPolygon() const {
  return polygon_;
}

SceneNode::SceneNode(Graph* pGraph, int index): GraphNode(pGraph, index) {
  pGraph->InsertSceneNode(this);
}

SceneNode::SceneNode(Graph* pGraph, int index, Polyline polyline) : GraphNode(pGraph, index), polyline_(polyline) {
  pGraph->InsertSceneNode(this);
}

SceneNode::SceneNode(const SceneNode& other): GraphNode(other),
  polyline_(other.polyline_) {}

SceneNode& SceneNode::operator = (const SceneNode& other) {
  GraphNode::operator=(other);
  polyline_ = other.polyline_;
  return *this;
}

void SceneNode::AddRoad(GraphNode* pNode) {
  RoadNode* road_node = dynamic_cast<RoadNode*>(pNode);
  Polyline polyline = road_node->GetPolygon().GetCenterLine();
  if (polyline_.NPoints() == 0) {
    polyline_ = polyline;
  }
  else {
    std::vector<Point> points = polyline.GetPoints();
    for (auto point: points) {
      polyline_.InsertPoint(point, 3);
    }
  }
  InsertChild(pNode);
}

std::vector<double> SceneNode::Value() const {
  std::vector<double> value;
  Polyline drive_direction = graph_->GetDriveDirection();
  // double length_ratio = polyline_.GetLength() / drive_direction.GetLength();
  // // if (length_ratio < 1) {
  // //   length_ratio = 1 / length_ratio;
  // // }
  // value.push_back(length_ratio);
  double angle = drive_direction.AngleBetweenPolyline(polyline_);
  // we might don't have enough samples of scenes. The density function in dimension of angle could be spicky. 
  double angle_epsilon = (rand() % 6 - 3);
  value.push_back(angle + angle_epsilon);

  Polygon polygon = graph_->GetPolygon();
  double length_ratio1 = polyline_.GetLength() / polygon.GetLeftline().GetLength();
  double length_ratio2 = polyline_.GetLength() / polygon.GetRightline().GetLength();
  if (length_ratio1 > length_ratio2) {
    value.push_back(length_ratio1);
    value.push_back(length_ratio2);
  }
  else {
    value.push_back(length_ratio2);
    value.push_back(length_ratio1);
  }
  return value;
}

std::vector<double> SceneNode::Relation(const GraphNode* pNode) const {
  return std::vector<double>(1, 0.0);
}

bool SceneNode::AddSibling(GraphNode* pNode) {
  return false;
}

int SceneNode::Position(GraphNode* pNode) const {
  return 0;
}

LineSegment SceneNode::GetLineSegment() const {
  Point point1(0, 0);
  Point point2(1, 1);
  return LineSegment{point1, point2};
}

GraphNode* SceneNode::ClosestGraphNode(std::vector<GraphNode*> siblings) const {
  return NULL;
}

Polyline SceneNode::GetPolyline() const {
  return polyline_;
}

/*
  graph might have several scene nodes. 
*/
void SceneNode::AddSuccessorsInLanes() {
  if (NChildren() < 2) {
    return;
  }
  std::vector<LaneNode*> lanes;
  for (auto road: Children()) {
    for (auto l: road->Children()) {
      LaneNode* lane_node = dynamic_cast<LaneNode*>(l);
      lanes.push_back(lane_node);
    }
  }
	for (std::vector<LaneNode*>::iterator lane1 = lanes.begin(); lane1 != lanes.end(); lane1 ++) {
	  for (std::vector<LaneNode*>::iterator lane2 = lane1 + 1; lane2 != lanes.end(); lane2 ++) {
      (*lane1)->CheckSuccessor(*lane2);
	  }
	}
}

/*
  Check the completeness of the scene node
*/
bool SceneNode::CheckSemantic() {
  int n_road = NChildren();
  int n_lanes = 0;
  int n_lane_boundaries = graph_->GetLaneBoundaries().size();
  int n_boundaries = graph_->GetBoundaries().size();
  std::set<GraphNode*> lane_boundaries;
  for (auto road: children_) {
    std::vector<GraphNode*> road_children = road->Children();
    n_lanes += road_children.size();
    for (auto lane: road_children) {
      std::vector<GraphNode*> lane_children = lane->Children();
      for (auto lane_boundary: lane_children) {
        lane_boundaries.insert(lane_boundary);
      }
    }
  }
  // if (lane_boundaries.size() != n_lane_boundaries) {
  //   utility::Print("missing some lane boundaries");
  //   return false;
  // }
  // if (n_road == 1) {
  //   if ((n_lanes + 1) != n_boundaries) {
  //     return false;
  //   }
  // }
  // else if (n_road = 2) {
  //   if ((n_lanes + 3) != 2 * n_boundaries) {
  //     return false;
  //   }
  // }
  // else {
  //   return false;
  // }
  return true;
}

} //ns
