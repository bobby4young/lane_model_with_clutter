#include <algorithm>    // std::find
#include <vector>
#include <iostream>
#include "Grammar.h"
#include "GraphNode.h"
#include "Label.h"
#include "Rule.h"

namespace lane_model {

RuleInstance::RuleInstance(const std::vector<Label*>& suc_label_array,
					const std::vector<GraphNode*>& suc_array,
					GraphNode* pNode)
					:label_instances_(suc_label_array),
					node_(pNode) {
	for (int i=0; i<(int)suc_label_array.size(); i++) {
    Label* pLabel1 = suc_label_array[i];
    if (!pLabel1) continue;
    for (int j=0; j<(int)suc_label_array.size(); j++) {
      if (i == j) continue;
      Label* pLabel2 = suc_label_array[j];
      if (!pLabel2) continue;
			// e.g. ("marking","marking")
      std::pair<Label*, Label*> labelPair = std::make_pair(pLabel1, pLabel2); 
      if (descriptors_.find(labelPair) == descriptors_.end()){
        // new pair
				descriptors_[labelPair] = std::vector<PairwiseDistance>();
			}
      PairwiseDistance pwDist = suc_array[i]->Relation(suc_array[j]);
      descriptors_[labelPair].push_back(pwDist);
    }
	}
}

Rule::Rule(Grammar* pGrammar): index_(-1), grammar_(NULL), predecessor_(NULL), fix_successor_cardinality_(false) {
	if (pGrammar) pGrammar->InsertRule(this);
	cardinality_ = std::set<int>();
}

void Rule::InitializeFromLabelArray(const std::vector<Label*>& label_array) {
	std::cout << "Initializing Rule FromLabelArray with size " << label_array.size() << std::endl; 
	predecessor_ = label_array[label_array.size()-1];
	successors_ = label_array;
	successors_.pop_back();
	std::cout << "Initialized FromLabelArray with size " << successors_.size() << std::endl;
	//SortSuccessorsByIndex();
}

void Rule::AddLabeledNode(GraphNode* pNode, const std::vector<Label*>& successor_labels, const std::vector<GraphNode*>& successor_nodes) {
	// successor_labels provides what the current label is.
	// successor_nodes provides the descriptor
	RuleInstance* pRuleInstance = new RuleInstance(successor_labels, successor_nodes, pNode);
	rule_instances_.push_back(pRuleInstance);
}

Grammar* Rule::GetGrammar() const {
	return grammar_;
}
Label* Rule::Predecessor() const {
	return predecessor_;
}

int Rule::NSuccessors() const {
	return successors_.size();
}

Label* Rule::Successor(int i) const {
	assert(i > 0 || i < successors_.size());
	return successors_[i];
}

std::string Rule::ToString() const {
	std::string pred_name = Predecessor()->BasicCategoryName();
  std::string succ_name;
  for (int i = 0; i < NSuccessors(); i++) {
    if (i == NSuccessors() - 1) {
      succ_name = succ_name + Successor(i)->BasicCategoryName();
      break;
    }
    succ_name = succ_name + Successor(i)->BasicCategoryName() + " ";
  }
	std::string rule = pred_name + " -> " + succ_name;
	return rule; 
}

void Rule::ClearInstances() {
  std::cout << "rule_instances_ .size()"<< rule_instances_.size() << std::endl;
	for (int i=0; i<(int)rule_instances_.size(); i++) {
		delete rule_instances_[i];
	}
  std::cout << "begin ClearRuleInstances " << std::endl;
	rule_instances_.clear();
  std::cout << "begin ClearRuleInstances " << std::endl;
}

int Rule::Index() const {
	return index_;
}

bool Rule::operator==(Rule rule) const {
	return ToString() == rule.ToString();
}

bool Rule::HasSuccessor(Label* pLabel) const {
  if (std::find (successors_.begin(), successors_.end(), pLabel) == successors_.end()) {
		return false;
	}
  else {
		return true;
	}
}

std::vector<Label*> Rule::Successors() const {
	return successors_;
}

void Rule::SetCardinality(std::set<int> cardinality) {
	cardinality_ = cardinality;
}

std::set<int> Rule::GetCardinality() const {
	return cardinality_;
}

void  Rule::SetSuccessorCardinalityFixed() {
	fix_successor_cardinality_ = true;
}

bool  Rule::IsFixSuccessorCardinality() const {
	return fix_successor_cardinality_;
}

} //ns
