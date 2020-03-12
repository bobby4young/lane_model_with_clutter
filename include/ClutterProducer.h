#ifndef LANE_MODEL_CLUTTER_PRODUCER_H_
#define LANE_MODEL_CLUTTER_PRODUCER_H_

#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>
#include "Graph.h"

namespace lane_model {

class Grammar;
class MarkingNode;
class BoundaryNode;
class Grammar;
class ClutterProducer {
 public:
  void SetGrammar(Grammar* grammar);
  void ImportTerminals(const std::string filename);
  /*
  category of clutter:
  1 -> random move and rotate the duplicated markings
  2 -> move the duplicated markings two meters to the right of driving direction
  3 -> remove one marking from each boundary
  4 -> rotate markings randomly between  -5 ~ 5 degree
  */
  void ProduceDataWithClutter(const std::string& dirname, int i,const int category);
 private:
  std::vector<MarkingNode*> markings_;
  std::vector<BoundaryNode*> boundaries_;
  Graph graph_;
  Grammar* grammar_;
  json json_graph_;
  double offset_x_, offset_y_;
};

}
#endif //LANE_MODEL_CLUTTER_PRODUCER_H_