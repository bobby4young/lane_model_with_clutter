#ifndef LANE_MODEL_RULE_H_
#define LANE_MODEL_RULE_H_
#include <map>
#include <set>
#include <vector>
#include <string>
#include <utility>

namespace lane_model {

class GraphNode;
class Label;
//class Descriptor;
class RuleInstance {
  typedef std::pair<Label*, Label*> LabelPair; 
  typedef std::vector<double> PairwiseDistance;
 public:
  RuleInstance(const std::vector<Label*>& suc_label_array,
  					const std::vector<GraphNode*>& suc__array,
  					GraphNode* pNode);
 private: 
  GraphNode* node_;
  std::map<LabelPair, std::vector<PairwiseDistance> > descriptors_;
  std::vector<Label*> label_instances_;
};

class Rule {
  typedef std::pair<Label*, Label*> LabelPair;
  typedef std::vector<double> PairwiseDistance;
  friend class Grammar;
 public:
  Rule(Grammar* pGrammar);
  void InitializeFromLabelArray(const std::vector<Label*>& label_array);
  //add node to rule instances
  void AddLabeledNode(GraphNode* pNode, const std::vector<Label*>& successor_labels, const std::vector<GraphNode*>& successor_nodes); 
  Grammar* GetGrammar() const;
  Label* Predecessor() const;
  Label* Successor(int i) const;
  std::vector<Label*> Successors() const;
  int NSuccessors() const;
  std::string ToString() const;
  void ClearInstances();
  int Index() const;
  bool operator==(Rule rule) const;
  bool HasSuccessor(Label* pLabel) const;
  void SetCardinality(std::set<int> cardinality);
  std::set<int> GetCardinality() const;
  void SetSuccessorCardinalityFixed();
  bool IsFixSuccessorCardinality() const;
 private:
  int index_;
  bool fix_successor_cardinality_;
  Grammar* grammar_;
  Label* predecessor_;
  std::vector<Label*> successors_;
  std::set<int> cardinality_;
  std::vector<RuleInstance*> rule_instances_;
};

} //ns
#endif