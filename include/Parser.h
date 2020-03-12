#ifndef LANE_MODEL_PARSER_H_
#define LANE_MODEL_PARSER_H_
#include <chrono> 
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>
#include "Graph.h"

namespace lane_model {

class Grammar;
class Label;
class MarkingNode;
class BoundaryNode;
class LaneBoundaryNode;
class LaneNode;
class RoadNode;
class SceneNode;

class Parser {
 public:
  Parser();
  ~Parser();
  void SetGrammar(Grammar* pGrammar);
  void Import(std::string& dirname);
  void ImportTerminals(std::string dirname, const int clutter_type);
  void ComputeGraphsEnergy();
  double ComputeGraphEnergy(GraphNode* graph_node);
  void ComputeRuleEnergyRecursive(GraphNode* pGraphNode, double& energy);
  void PickBestGraph(const int clutter_type);
  void PrintGraph(GraphNode* node) const;
  void CombinationParse();

  std::vector<BoundaryNode*> BFSAddBoundary();
  bool BFSAddOneBoundary(std::vector<bool> &visited, std::vector<MarkingNode*>::iterator i, BoundaryNode* &out);
  // separate CombinationParse
  void GenerateEdgeInMarkings();
  void GenerateBoundaries();
  void GenerateLaneBoundaries();
  void GenerateLanes();
  void GenerateRoads();
  void GenerateScenes();
  void BottomUpParse();

  void Log(const std::chrono::milliseconds duration, const int graph_index, const int clutter_type) const;
  void SetVisual(const bool do_visual);
 private:
  // arrays holds potential nodes. Don't need "delete". Delete is taken care by graph's destructor
  std::vector<MarkingNode*> markings_;
  std::vector<BoundaryNode*> boundaries_;
  std::vector<LaneBoundaryNode*> lane_boundaries_;
  std::vector<LaneNode*> lanes_;
  std::vector<RoadNode*> roads_;
  std::vector<SceneNode*> scenes_;

  int n_leaves_node_; 
  Label* root_label_;
  Grammar* grammar_;
  // input graph 
  Graph graph_, base_graph_;
  std::string file_path_;
  std::vector<Graph*> candidate_graphs_;
  std::vector<bool> complete_graph_boolean_;
  std::vector<double> energy_array_;
  bool is_visual_;
};

}
#endif //LANE_MODEL_PARSER_H_