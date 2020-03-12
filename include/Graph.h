#ifndef LANE_MODEL_GRAPH_H_
#define LANE_MODEL_GRAPH_H_
#include <vector>
#include <unordered_map>
#include "ObjectShape.h"
#include "json.hpp"

using json = nlohmann::json;

namespace lane_model {
class GraphNode;
class MarkingNode;
class BoundaryNode;
class LaneBoundaryNode;
class LaneNode;
class RoadNode;
class SceneNode;
class Grammar;

class Graph {
 public:
  friend class GraphNode;
  friend class RoadNode;
  Graph();
  Graph(int graph_index);
  // TODO: add copy constructor copy assignment operator function of class Graph
  Graph(const Graph& other);
  Graph& operator = (const Graph& other);
  ~Graph();
  void ImportGraph(const std::string& filename, int i);
  void InsertNode(GraphNode* pNode);
  void InsertMarkingNode(MarkingNode* pNode);
  void InsertBoundaryNode(BoundaryNode* pNode);
  void InsertLaneBoundaryNode(LaneBoundaryNode* pNode);
  void InsertLaneNode(LaneNode* pNode);
  void InsertRoadNode(RoadNode* pNode);
  void InsertSceneNode(SceneNode* pNode);
	void AddGraphBoundary(Polyline polyline);
  double GetGraphWidth() const;
  double GetGraphWidthAtPoint(Point point) const;
  Polygon GetPolygon() const;
  std::string ToString() const;

	GraphNode* Root(void) const;
	GraphNode* Node(int k) const;
  GraphNode* GetNodeWithIndex(int k) const;

  void EraseNode(const GraphNode* pNode);
  void EraseNodePointer(const GraphNode* pNode);
  void EraseMarkingNode(const MarkingNode* pNode);
  void EraseBoundaryNode(const BoundaryNode* pNode);
  void EraseLaneNode(const LaneNode* pNode);
  void EraseRoadNode(const RoadNode* pNode);
  void EraseSceneNode(const SceneNode* pNode);
	void GenerateDriveDirection();
  Polyline GetDriveDirection() const;
  int NNodes(void) const;
  int NRoads() const;
  void SetGrammar(Grammar* pGrammar);
  void SetRoot(GraphNode* pNode);
  Grammar* GetGrammar(void) const;

  int GraphIndex() const;
  void SetGraphIndex(const int index);
  double SceneSize() const;
  void SetSceneSize(double size);
  /* 
    merge and split
  */
  // vector of boundary nodes after splitting by this line segment 
  json ImportUnlabeledGraph(const std::string& filename, int i);
  //std::vector<BoundaryNode*> SplitBoundary(const LineSegment line_segment);
  void SplitBoundary(const LineSegment line_segment);
  // the number of this line segment intersects with boundaries in the graph
  int NIntersectWithLine(LineSegment line_segment);
  // Is any boundary intersect with another boundary
  bool IsBoundariesIntersect() const;
  // generate new boundary nodes, return n split
  int GenerateLaneBoundary();
  void ProduceLabeledData(const std::string& dirname, int i, json& json_graph);
  int NBoundaries() const;
  std::vector<MarkingNode*> GetMarkings() const;
  std::vector<BoundaryNode*> GetBoundaries() const;
  std::vector<LaneBoundaryNode*> GetLaneBoundaries() const;
  std::vector<LaneNode*> GetLanes() const;
  std::vector<RoadNode*> GetRoads() const;
  std::vector<SceneNode*> GetScenes() const;

  // export edge csv file
  void ExportMarkingEdgeData(std::ofstream& file) const;
  void ExportLaneBoundaryEdgeData(std::ofstream& file) const;

 private:
  int graph_index_;
  // size of the local scene in meters
  double scene_size_;
  double offset_x_;
  double offset_y_;
  // n_roads_ is not size of roads_, but (number of boundary splitting  + 1)
  int n_roads_;
  /* 
    driving_direction_: a polyline indicates the driving direction in current road
    derived from center line of two road boundaries
  */
  Polyline driving_direction_; 
  std::vector<Polyline> graph_boundaries_;
  Polygon polygon_;
  // distance between the two graph boundaries
  double graph_width_;
  GraphNode* root_;
  Grammar* grammar_;
  std::vector<GraphNode*> nodes_;
  std::unordered_map<int, GraphNode*> nodes_map_;

  std::vector<MarkingNode*> markings_;
  std::vector<BoundaryNode*> boundaries_;
  std::vector<LaneBoundaryNode*> lane_boundaries_;
  std::vector<LaneNode*> lanes_;
  std::vector<RoadNode*> roads_;
  std::vector<SceneNode*> scenes_;

};
} //ns
#endif  // LANE_MODEL_GRAPH_H_
