#ifndef LANE_MODEL_GRAPH_NODE_H_
#define LANE_MODEL_GRAPH_NODE_H_

#include <map>
#include <vector>
#include <set>
#include <string>
#include "ObjectShape.h"

namespace lane_model {

class Graph;
class Label;
class Rule;

class GraphNode {
  friend class Graph;
 public: 
  GraphNode(Graph* pGraph);
  GraphNode(Graph* pGraph, int index);
  GraphNode(const GraphNode& other);
  GraphNode& operator = (const GraphNode& other);
  virtual ~GraphNode(){};
  int NChildren(void) const;
  GraphNode* Child(int i) const;
  GraphNode* ParentNode() const;
  // TODO  delete this function
  std::vector<GraphNode*> Children() const;
  
  void InsertChild(GraphNode* pNode);
  bool IsTerminal() const;
  void SetAssociatedLabel(Label* pLabel);
  void SetAssociatedRule(Rule* rule);
  Label* AssociatedLabel(void) const;
  Graph* MyGraph(void) const;
  Rule* AssociatedGrammarRule() const;
  Rule* AssociatedRule() const;
  int GraphIndex() const;
  std::string ToString() const;
  bool operator == (GraphNode* pNode) const;

  bool IsSibling(GraphNode* pNode) const;
  bool HasSiblingInChildren(GraphNode* pNode) const;
  bool HasSameChild(GraphNode* pNode) const;
  void PrintSibling() const;
  virtual int Position(GraphNode* pNode) const = 0;
  virtual LineSegment GetLineSegment() const = 0;
  virtual bool AddSibling(GraphNode* pNode) = 0 ;
  virtual GraphNode* ClosestGraphNode(std::vector<GraphNode*> siblings) const = 0;
  virtual std::vector<double> Value() const = 0;
  virtual std::vector<double> Relation(const GraphNode* pNode) const = 0;
  
 protected:
  /* 
    Graph will take care of deleting all its nodes
    Label and Rule will be deleted by Grammar
    Graph will be deleted by GrammarProducer
  */
  std::vector<GraphNode*> children_;
  GraphNode* parent_;
  std::set<GraphNode*> siblings_;
	Graph* graph_;
	Label* label_;
	Rule* rule_;
 
 private:
  int graph_index_;
};

class MarkingNode: public GraphNode {
 public:
  MarkingNode(Graph* pGraph, int index);
  MarkingNode(Graph* pGraph, int index, Box bounding_box);
  MarkingNode(const MarkingNode& other);
  MarkingNode& operator = (const MarkingNode& other);
  ~MarkingNode(){};
	void SetBox(cv::RotatedRect cv_rect);
  LineSegment GetLineSegment() const;
  std::vector<double> Value() const override;
  std::vector<double> Relation(const GraphNode* pNode) const override;
  bool AddSibling(GraphNode* pNode);
  int Position(GraphNode* pNode) const ;
  GraphNode* ClosestGraphNode(std::vector<GraphNode*> siblings) const;
  Box GetBox() const;
 private:
  Box bounding_box_;
};

class BoundaryNode: public GraphNode {
 public:
  BoundaryNode(Graph* pGraph, int index);
  BoundaryNode(Graph* pGraph, int index, Polyline polyline);
  BoundaryNode(const BoundaryNode& other);
  BoundaryNode& operator = (const BoundaryNode& other);
  ~BoundaryNode(){};
  Polyline GetPolyline() const;
  LineSegment GetLineSegment() const;
  void AddMarking(GraphNode* pNode);
  void RemoveMarking(GraphNode* pNode);
  int Position(GraphNode* pNode) const;
  std::vector<double> Value() const override;
  std::vector<double> Relation(const GraphNode* pNode) const override;
  bool AddSibling(GraphNode* pNode);
  GraphNode* ClosestGraphNode(std::vector<GraphNode*> siblings) const;
  std::pair<Polyline, Polyline> Split(Point point);
  bool IsIntersect(const LineSegment line_segment) const;
  bool IsIntersect(const BoundaryNode* other_boundary) const;
  bool IsSubset(BoundaryNode* boundary_node) const;
 private:
  Polyline polyline_;
};

class LaneBoundaryNode: public GraphNode {
 public:
  LaneBoundaryNode(Graph* pGraph, int index);
  LaneBoundaryNode(Graph* pGraph, int index, Polyline polyline);
  LaneBoundaryNode(const LaneBoundaryNode& other);
  LaneBoundaryNode& operator = (const LaneBoundaryNode& other);
  ~LaneBoundaryNode(){};
  Polyline GetPolyline() const;
  LineSegment GetLineSegment() const;
  int Position(GraphNode* pNode) const;
  std::vector<double> Value() const override;
  std::vector<double> Relation(const GraphNode* pNode) const override;
  bool AddSibling(GraphNode* pNode);
  GraphNode* ClosestGraphNode(std::vector<GraphNode*> siblings) const;
  std::pair<Polyline, Polyline> Split(Point point);
  bool IsIntersect(LineSegment line_segment) const;
  void AddSuccessor(LaneBoundaryNode* successor);
  bool IsSuccessor(LaneBoundaryNode* lane_boundary_node) const;
 private:
  LaneBoundaryNode* successor_;
  Polyline polyline_;
};

class LaneNode: public GraphNode {
 public:
  LaneNode(Graph* pGraph, int index);
  LaneNode(Graph* pGraph, int index, Polygon polygon);
  LaneNode(const LaneNode& other);
  LaneNode& operator = (const LaneNode& other);
  ~LaneNode(){};
  void AddBoundary(GraphNode* pNode);
  LineSegment GetLineSegment() const;
  std::vector<double> Value() const;
  std::vector<double> Relation(const GraphNode* pNode) const;
  bool AddSibling(GraphNode* pNode);
  bool CheckAddSibling(LaneNode* lane_node);
  int Position(GraphNode* pNode) const;
  GraphNode* ClosestGraphNode(std::vector<GraphNode*> siblings) const;
  Polygon GetPolygon() const;
  Polyline GetCenterline() const;
  Point GetCenterPoint() const;
  bool IsSuccessor(LaneNode* lane_node) const;
  LaneNode* GetSuccessor() const;
  void AddSuccessor(LaneNode* successor);
  void CheckSuccessor(LaneNode* lane_node);
 private:
  // polygon_.first is the line on the left. second on the right
  Polygon polygon_;
  Polyline centerline_;
  //LineSegment centerline_line_segment_;
  // TODO not sure if box and point is neccesary
  Box bounding_box_;
  std::vector<Point> vertices_;
  //Point center_point_;
  LaneNode* successor_;
};

class RoadNode: public GraphNode {
 public:
  RoadNode(Graph* pGraph, int index);
  RoadNode(Graph* pGraph, int index, Polygon polygon);
  RoadNode(const RoadNode& other);
  RoadNode& operator = (const RoadNode& other);
  ~RoadNode(){};
  LineSegment GetLineSegment() const;
  void AddLane(GraphNode* pNode);
  std::vector<double> Value() const;
  std::vector<double> Relation(const GraphNode* pNode) const;
  bool AddSibling(GraphNode* pNode);
  int Position(GraphNode* pNode) const;
  GraphNode* ClosestGraphNode(std::vector<GraphNode*> siblings) const;
  Polygon GetPolygon() const;
 private:
  Polygon polygon_;
  Polyline centerline_;
  Point center_point_;
  RoadNode* successor_;
};

class SceneNode: public GraphNode {
 public:
  SceneNode(Graph* pGraph, int index);
  SceneNode(Graph* pGraph, int index, Polyline polyline);
  SceneNode(const SceneNode& other);
  SceneNode& operator = (const SceneNode& other);
  ~SceneNode(){};
  LineSegment GetLineSegment() const;
  void AddRoad(GraphNode* pNode);
  std::vector<double> Value() const;
  std::vector<double> Relation(const GraphNode* pNode) const;
  bool AddSibling(GraphNode* pNode);
  int Position(GraphNode* pNode) const;
  GraphNode* ClosestGraphNode(std::vector<GraphNode*> siblings) const;
  Polyline GetPolyline() const;
  void AddSuccessorsInLanes();
  bool CheckSemantic();
 private:
  Polyline polyline_;  
};

} //ns
#endif  // LANE_MODEL_GRAPH_NODE_H_
