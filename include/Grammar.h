#ifndef LANE_MODEL_GRAMMAR_H_
#define LANE_MODEL_GRAMMAR_H_

#include <algorithm>
#include <map>
#include <utility>
#include <vector>
#include <set>

namespace lane_model {

class Label;
class Rule;
class LabelEdge;
class Classifier;
class Graph;
class GraphNode;

class Grammar {
 public:
  Grammar();
  ~Grammar();
  // I/O
  void Import(const std::string& dirname);
  void Export(const std::string& dirname) const;
  void ImportLabels(const std::string& filename);
  void ImportRules(const std::string& filename);
  void ExportLabels(const std::string& dirname) const;
  void ExportRules(const std::string& dirname) const;
  void ExportLabelEdges(const std::string& dirname) const;
  void ExportClassifier(const std::string& dirname) const;

	// labels
  int NLabels(void) const;
  void InsertLabel(Label* pLabel);
	Label* GetLabelByIndex(int k) const;
	Label* GetLabelByString(const std::string& str) const;
	Label* RootLabel() const;
  void SetRootLabel(Label* root);
  void ClearLabels();
  bool IsLabelTerminal(Label* label) const;
  // rules
  int NRules(void) const;
  void InsertRule(Rule* rule);
  Rule* GetRuleByIndex(int k) const;
  std::vector<Rule*> GetRulesDerivedFromLabel(Label* pLabel) const;
  void ClearRules();
  void ComputeDerivedRuleMap(void);
	//label edges
  int NEdges(void) const;
  LabelEdge* GetLabelEdgeByIndex(int k) const;
  LabelEdge* GetLabelEdgeByInstance(Rule* pRule, Label* pLabel1, Label* pLabel2) const;
  LabelEdge* GetLabelEdgeByString(const std::string& name) const;
  std::vector<LabelEdge*> GetLabelEdgesByRule(Rule *pRule) const;
  void CreateEdges();
  void InsertLabelEdge(LabelEdge* pLabelEdege);
  void ClearLabelEdges();
  void BuildLabelEdgeTableForQuickQuery();
  void ExportTrainingClassifierSingleEdge(LabelEdge* pEdge, std::string dirname) const;
  // add and clear training data
	void AddLabeledGraph(Graph* pGraph);
  void AddLabeledGraphForLabels(Graph* pGraph);
  void AddLabeledGraphForRules(Graph* pGraph);
  void AddLabeledGraphForLabelEdges(Graph* pGraph);
  void AddLabeledNodeForLabelEdge(std::vector<Label*>child_labels, std::vector<GraphNode*> children_nodes, Rule* pRule);
  void AddKernels();
	void ClearInstances(void);
  void ClearLabelInstances(void);
  void ClearRuleInstances(void);
  void ClearLabelEdgeInstances(void);
  
 private:
  std::vector<Label*> labels_;
  std::vector<Rule*> rules_;
  std::vector<LabelEdge*> label_edges_;
  std::map<Label*, std::vector<Rule*> > derived_rule_map_;
  std::map<std::pair<Label*, Label*>, std::vector<Rule*> > deriving_rule_map_;
  // pairwise distance between labels (test for cycles)
  std::vector<std::vector<int> > DistanceTable_;
  std::vector<std::vector<int> > siblings_array;
  // terminal instances of each label
  std::vector<int> NumTerminalInstances_;
  // classifier
  Classifier* classifier_;
  // label edge related
  // index 0: rule index
  // index 1: label 1 index
  // index 2: label 2 index
  std::vector< std::vector< std::map<Label*, LabelEdge*> > > label_edge_table_;
  Label* root_label_;
  //LabelEdge* MeaninglessLabelEdge_;
};

} //ns
#endif  // LANE_MODEL_GRAMMAR_H_
