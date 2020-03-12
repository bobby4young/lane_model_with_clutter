#ifndef LANE_MODEL_LABEL_H_
#define LANE_MODEL_LABEL_H_

#include <vector>
#include <string>
#include "GraphNode.h"

namespace lane_model {

class Grammar;
class GaussianKernel;
//lass Descriptor;

class LabelInstance {
 public:
	LabelInstance(GraphNode* pNode);
  const std::vector<double> GetValue() const;
 private:
  //Descriptor* descriptor_;
	int associated_node_index_;
  GraphNode* node_;
};

class Label {
  friend class Grammar;
 public:
  Label(Grammar* pGrammar);
  void SetBasicCategory(const std::string& category);
  std::string BasicCategoryName() const;
  void ClearInstances();
  void AddLabeledNode(GraphNode* pNode);
  Grammar* GetGrammar() const;
  int Index() const;
  void AddLabelKernel();
  int NInstances() const;
  double ComputeEnergy(std::vector<double> sample) const;
  bool IsLabel(GraphNode* pNode) const;
  bool IsMinimalLabel(GraphNode* pNode) const;

 private:
  int index_;
  bool is_terminal_;
  Grammar* grammar_;
  std::string basic_category_;
  std::vector<LabelInstance*> label_instances_;
  GaussianKernel* gaussian_kernel_;
  double minimum_likelihood_;
  double average_likelihood_;
};

}
#endif  // LANE_MODEL_LABEL_H_
