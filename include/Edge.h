#ifndef LANE_MODEL_LABEL_EDGE_H_
#define LANE_MODEL_LABEL_EDGE_H_
#include <vector>
#include <string>

namespace lane_model {
//class Graph;
class Label;
class Rule;
class GraphNode;
class Grammar;
class GaussianKernel;

class LabelEdgeInstance {
 public:
  LabelEdgeInstance(GraphNode* pNode1,GraphNode* pNode2,
                      const std::vector<double>& distance);
  const std::vector<double>& GetPairwiseDistance() const;
  const std::vector<double>& GetDiameter1() const;
  const std::vector<double>& GetDiameter2() const;
 private:
  GraphNode* node_1_;
  GraphNode* node_2_; 
  std::vector<double> relation_;
};

class LabelEdge {
  friend class Grammar;
 public:
  LabelEdge(Grammar* pGrammar);
  ~LabelEdge();
  void ClearInstances();
  void SetAssociatedRuleAndLabels(Rule* pRule, Label* pLabel_1, Label* pLabel_2);
  const std::string ToString() const;
  Rule* GetGrammarRule() const;
  Label* GetLabel_1() const;
  Label* GetLabel_2() const;
  int Index() const;
  int NInstances() const;
  void AddLabeledNode(GraphNode* pNode1, GraphNode* pNode2);
  void AddEdgeKernel();
  double ComputeEnergy(std::vector<double> sample) const;
  bool IsEdge(GraphNode* pNode1, GraphNode* pNode2) const;
  bool operator==(LabelEdge edge) const;
  void GenerateVisualizationFile() const;

 private:
  int index_;
  Grammar* grammar_;
  Rule* rule_;
  Label* label_1_;
  Label* label_2_;
  std::vector<LabelEdgeInstance*> edge_instances_;
  bool enough_instances_;
  GaussianKernel* gaussian_kernel_;
  double minimum_likelihood_;
  double average_likelihood_;
  /* 
    To visualize specific data, three variables are to be changed.
    label_name_ in the constructor
    index1 and index2 in function GenerateVisualizationFile
  */
  std::string label_name_;
  std::vector<std::vector<double> > visual_relation_;
};

} //ns
#endif //LANE_MODEL_LABEL_EDGE_H_
