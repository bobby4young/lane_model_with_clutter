#include <assert.h>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <string.h>
#include "Classifier.h"
#include "Grammar.h"
#include "Label.h"
#include "Edge.h"
#include "Graph.h"
#include "GraphNode.h"
#include "Rule.h"
#include "Utility.h"

#include "json.hpp"

using json = nlohmann::json;

namespace lane_model {

Grammar::Grammar(): root_label_(NULL) {}

Grammar::~Grammar() {
  ClearLabels();
  ClearLabelEdges();
  ClearRules();
}

void Grammar::Import(const std::string& dirname) {
  // TODO
  // grammar should be imported, so that don't need to be learnt and trained every time
}

void Grammar::Export(const std::string& dirname) const {
  std::cout << "begin export" << std::endl;
  ExportLabels(dirname);
  ExportRules(dirname);
  ExportLabelEdges(dirname);
  ExportClassifier(dirname);
}

void Grammar::ExportLabels(const std::string& dirname) const {
  std::cout << "begin export label" << std::endl;
  json label_json;
  for (int i = 0; i < NLabels(); i++) {
    label_json[std::to_string(i)] = {
      {"basic_category", GetLabelByIndex(i)->BasicCategoryName() }
    };
  }
  std::string file_name = dirname + "label.json";
  std::ofstream o(file_name);
  o << std::setw(4) << label_json << std::endl;
  o.close();
}

void Grammar::ExportRules(const std::string& dirname) const {
  std::cout << "begin export rule" << std::endl;
  json rule_json;
  for (int i = 0; i < NRules(); i++) {
    Rule* pRule = GetRuleByIndex(i);
    if (pRule == NULL) {
      std::cout << "RULE " << i << " is NULL" << std::endl;
      continue;
    }
    std::string pred = std::to_string(pRule->Predecessor()->index_);
    std::string succ;
    for (int i = 0; i < pRule->NSuccessors(); i++) {
      if (i == pRule->NSuccessors() - 1) {
        succ = succ + std::to_string(pRule->Successor(i)->index_);
        break;
      }
      succ = succ + std::to_string(pRule->Successor(i)->index_) + " ";
    }
    std::string production_rule = pred + "->" + succ;
    std::string text_rule = pRule->ToString();
    
    std::string cardinality;
    for (auto number: pRule->GetCardinality()) {
      cardinality += (std::to_string(number) + " ");
    }
    rule_json[std::to_string(i)] = {
      {"production_rule", production_rule},
      {"text_rule", text_rule},
      {"cardinality", cardinality}
    };
  }
  std::string file_name = dirname + "rule.json";
  std::ofstream out(file_name);
  out << std::setw(4) << rule_json << std::endl;
  out.close();
  std::cout << "finish export rule" << std::endl;
}

void Grammar::ExportLabelEdges(const std::string& dirname) const {
  json edge_json;
  for (int i = 0; i < NEdges(); i++) {
    LabelEdge* label_edge = GetLabelEdgeByIndex(i);
    if (label_edge == NULL) {
      std::cout << "label_edge " << i << " is NULL" << std::endl;
      continue;
    }
    edge_json[std::to_string(i)] = {label_edge->ToString()};
  }
  std::string file_name = dirname + "edge.json";
  std::ofstream out(file_name);
  out << std::setw(4) << edge_json << std::endl;
  out.close();
  std::cout << "finish export edge" << std::endl;
}

void Grammar::ExportClassifier(const std::string& dirname) const {
  for (int i = 0; i < NEdges(); i++) {
    LabelEdge* pEdge = GetLabelEdgeByIndex(i);
    std::string name = pEdge->ToString();
    classifier_->ExportTraining(pEdge, dirname);
    //name = StringUtil::Replace(name, ' ', '_');
    //ExportTrainingClassifierSingleEdge(pEdge, dirname);
  }
}

void Grammar::ExportTrainingClassifierSingleEdge(LabelEdge* pEdge, std::string dirname) const {

}
int Grammar::NLabels(void) const {
	return int(labels_.size());
}

int Grammar::NRules(void) const {
	return int(rules_.size());
}

int Grammar::NEdges(void) const {
	return int(label_edges_.size());
}

Label* Grammar::GetLabelByIndex(int k) const {
  if (k<0 || k>=NLabels()){
    std::cout << "return NULL Label" << std::endl;
    return NULL;
  }
  return labels_[k];
}

Label* Grammar::GetLabelByString(const std::string& str) const {
  //std::cout << "number of label " << labels_.size() << std::endl;
  for (int i = 0; i < labels_.size(); i++) {
  	Label* label = labels_[i];
  	if (label->BasicCategoryName() == str) return label;
  }
  std::cout << "wanted label " << str << ". couln't find it, return NULL Label" << std::endl;
  return NULL;
}

void Grammar::SetRootLabel(Label* root_label) {
  root_label_ = root_label;
}

Label* Grammar::RootLabel() const {
  return root_label_;
}

void Grammar::InsertLabel(Label* pLabel) {
  assert(!pLabel->grammar_);
	assert(pLabel->index_ == -1);
	pLabel->grammar_ = this;
	pLabel->index_ = int(labels_.size());
	labels_.push_back(pLabel);
}

void Grammar::ClearLabels() {
  for (int i=0; i<int(labels_.size()); i++) {
    delete labels_[i];
  }
	labels_.clear(); 
}

bool Grammar::IsLabelTerminal(Label* label) const {
  std::map<Label*, std::vector<Rule*> >::const_iterator it = derived_rule_map_.find(label);
  if (it != derived_rule_map_.end()) {
    if (it->second.size() == 0) {
      return true;
    }
    return false;
  }
  else {
    return false;
  }
}

Rule* Grammar::GetRuleByIndex(int k) const {
	if (rules_[k] == NULL) {
    std::cout << "return NULL RULE" << std::endl;
  }
	return rules_[k];
}

std::vector<Rule*> Grammar::GetRulesDerivedFromLabel(Label* pLabel) const {
	std::map<Label*, std::vector<Rule*> >::const_iterator it = derived_rule_map_.find(pLabel);
  if (it == derived_rule_map_.end()) {
    std::cout << "can't find rule of label " << pLabel->BasicCategoryName() << std::endl;
    std::vector<Rule*> rules(1, NULL);
    return rules;
  }
	return it->second;
}

void Grammar::ClearRules() {
  for (int i=0; i<int(rules_.size()); i++) {
    delete rules_[i];
  }
	rules_.clear();
}

void Grammar::InsertRule(Rule* pRule) {
	assert(!pRule->grammar_);
	assert(pRule->index_ == -1);
	pRule->grammar_ = this;
	pRule->index_ = int(rules_.size());
	rules_.push_back(pRule);
}

LabelEdge* Grammar::GetLabelEdgeByIndex(int k) const {
  assert(k>=0 && k<NEdges());
  return label_edges_[k];
}

LabelEdge* Grammar::GetLabelEdgeByInstance(Rule* pRule, Label* pLabel_1, Label* pLabel_2) const {
  if (!pRule || !pLabel_1 || !pLabel_2) {
      if (pRule) std::cout << "pRule = " << pRule->ToString() << std::endl;
      if (pLabel_1) std::cout << "pLabel_1 = " << pLabel_1->BasicCategoryName() << std::endl;
      if (pLabel_2) std::cout << "pLabel_2 = " << pLabel_2->BasicCategoryName() << std::endl;
      return NULL;
  }
  int rule_index = pRule->Index();
  int label_1_index = pLabel_1->Index();
  const std::map<Label*, LabelEdge*>& label_2_edge_Map = label_edge_table_[rule_index][label_1_index];
  std::map<Label*, LabelEdge*>::const_iterator it = label_2_edge_Map.find(pLabel_2);
  if (it == label_2_edge_Map.end()) {
    std::cout << "couldn't find one edge. Label 1: " << pLabel_1->BasicCategoryName() 
        << " Label 2: " << pLabel_2->BasicCategoryName() << " Rule " << pRule->Predecessor()->BasicCategoryName() << std::endl;
    return NULL;
  }
  //std::cout << "found one edge. Label 1: " << pLabel_1->BasicCategoryName() 
  //     << " Label 2: " << pLabel_1->BasicCategoryName() << std::endl;
  return it->second;
}

LabelEdge* Grammar::GetLabelEdgeByString(const std::string& name) const {
  for (int i = 0; i < label_edges_.size(); i ++) {
    LabelEdge* edge = label_edges_[i];
    std::string label1= edge->GetLabel_1()->BasicCategoryName();
    std::string label2= edge->GetLabel_2()->BasicCategoryName();
    if (label1 == label2 && label1 == name) {
      return edge;
    }
  }
  return NULL;
}

std::vector<LabelEdge*> Grammar::GetLabelEdgesByRule(Rule *pRule) const {
  std::vector<LabelEdge*> edges;
  for (int i = 0; i < NEdges(); i++) {
    LabelEdge* pEdge = GetLabelEdgeByIndex(i);
    if (pRule == pEdge->GetGrammarRule()) {
      edges.push_back(pEdge);
    }
  }
  return edges;
}

void Grammar::CreateEdges() {
  std::cout << "begin CreateLabelEdges " << std::endl;
  ClearLabelEdges();  
  for (int i = 0; i < NRules(); i++) {
    Rule* pRule = GetRuleByIndex(i);
    for (int j = 0; j < pRule->NSuccessors(); j++) {
      Label* pLabel1 = pRule->Successor(j);
      for (int k = 0; k< pRule->NSuccessors(); k++) {
        // label1 and label2 could be identical
        Label* pLabel2 = pRule->Successor(k);  
        LabelEdge* pEdge = new LabelEdge(this);
        pEdge->SetAssociatedRuleAndLabels(pRule, pLabel1, pLabel2);
      }
    }
  }
  BuildLabelEdgeTableForQuickQuery();
  std::cout << "finish create edges and build edge table " << NEdges() << std::endl;
}

void Grammar::InsertLabelEdge(LabelEdge* pLabelEdge) {
  pLabelEdge->grammar_ = this;
  pLabelEdge->index_ = (int)label_edges_.size();
  label_edges_.push_back(pLabelEdge);
}

void Grammar::ClearLabelEdges() {
  for (int i=0; i<int(label_edges_.size()); i++) {
    delete label_edges_[i];
  }
	label_edges_.clear();
}

void Grammar::BuildLabelEdgeTableForQuickQuery() {
  label_edge_table_ = std::vector< std::vector< std::map<Label*, LabelEdge*> > >();
  label_edge_table_.resize(NRules());
  for (int i = 0; i < label_edge_table_.size(); i++) {
    label_edge_table_[i].resize(NLabels());
  }
  // this routine can process
  // (1) normal edges
  // (2) meaningless edge
  // (3) uncertain edges
  for (int i = 0; i < NEdges(); i++) {
    LabelEdge* pEdge = GetLabelEdgeByIndex(i);
    Rule* pRule = pEdge->GetGrammarRule();
    Label* pLabel1 = pEdge->GetLabel_1();
    Label* pLabel2 = pEdge->GetLabel_2();
    label_edge_table_[pRule->Index()][pLabel1->Index()][pLabel2] = pEdge;
  }
  std::cout << "BuildLabelEdgeTableForQuickQuery: table size: " << label_edge_table_.size() <<" " 
      << label_edge_table_[0].size() << " " << label_edge_table_[0][0].size() << std::endl;
}

void Grammar::AddLabeledGraph(Graph* pGraph) {
  AddLabeledGraphForLabels(pGraph);
  AddLabeledGraphForRules(pGraph);
  AddLabeledGraphForLabelEdges(pGraph);
}

void Grammar::AddLabeledGraphForLabels(Graph* pGraph) {
  assert(pGraph->GetGrammar() == this);
	for (int i = 0; i < pGraph->NNodes(); i++) {
		GraphNode* pGraphNode = pGraph->Node(i);
		Label* pLabel = pGraphNode->AssociatedLabel();
    assert(pLabel);
		pLabel->AddLabeledNode(pGraphNode);
	}
  std::cout << "finish AddLabeledGraphForLabels" << std::endl;
}

void Grammar::AddLabeledGraphForRules(Graph* pGraph) {
  assert(pGraph->GetGrammar() == this);
	for (int i = 0; i< pGraph->NNodes(); i++) {
		GraphNode* pGraphNode = pGraph->Node(i);
    if (pGraphNode->NChildren() == 0) {
      continue;
    }
		// Add each instance of label
		Label* pLabel = pGraphNode->AssociatedLabel();
		// Add each instance of grammar rule
		Rule* pRule = pGraphNode->AssociatedRule();
		if (pRule) {
 			std::vector<Label*> child_label_array;
			std::vector<GraphNode*> child_node_array;
			for (int j = 0; j < pGraphNode->NChildren(); j++) {
				GraphNode* pChildGraphNode = pGraphNode->Child(j);
				Label* pChildLabel = pChildGraphNode->AssociatedLabel();
				child_label_array.push_back(pChildLabel);
				child_node_array.push_back(pChildGraphNode);
			}
			pRule->AddLabeledNode(pGraphNode, child_label_array, child_node_array);
		}
  }
  std::cout << "finish AddLabeledGraphForRules" << std::endl;
}

void Grammar::AddLabeledGraphForLabelEdges(Graph* pGraph) {
  std::cout << "begin AddLabeledGraphForLabelEdges " << std::endl;
  assert(pGraph->GetGrammar() == this);
  for (int i = 0; i < pGraph->NNodes(); i++) {
    GraphNode* pGraphNode = pGraph->Node(i);
    Rule* pRule = pGraphNode->AssociatedRule();
    if (pGraphNode->NChildren() == 0) continue;
    // if node only has one child, there is no edge between the children
    if (pGraphNode->NChildren() == 1) continue;
    std::vector<Label*> child_labels;
    std::vector<GraphNode*> children_nodes;
    for (int j = 0; j < pGraphNode->NChildren(); j++) {
        GraphNode* pChildNode = pGraphNode->Child(j);
        Label* pChildLabel = pChildNode->AssociatedLabel();
        child_labels.push_back(pChildLabel);
        children_nodes.push_back(pChildNode);
    }
    AddLabeledNodeForLabelEdge(child_labels, children_nodes, pRule);
  }
  std::cout << "finish AddLabeledGraphForLabelEdges" << std::endl;
}

void Grammar::AddLabeledNodeForLabelEdge(std::vector<Label*> child_labels, std::vector<GraphNode*> child_nodes, Rule* pRule) {
    // for each pair of children, we add the instance to the corresponding label edge
  if (child_nodes.size() == 1) return;
  for (int i = 0; i< (int)child_labels.size(); i++) {
    GraphNode* pNode1 = child_nodes[i];
    assert(pNode1);
    Label* pLabel1 = child_labels[i];
    /* 
      add all the siblings 
    */
    if (pLabel1->BasicCategoryName() == "lane") {
      /*  
          Besides the adjacent lane, also add the lanes which are not adjacent
          In this way, The likelihood of lane edge can be reliable when building road 
      */
      for (int j=0; j<(int)child_labels.size(); j++) {
          if (i == j) continue;
          GraphNode* pNode2 = child_nodes[j];
          assert(pNode2);
          Label* pLabel2 = child_labels[j];
          LabelEdge* label_edge = GetLabelEdgeByInstance(pRule, pLabel1, pLabel2);
          //ReportLabelEdgeNotFound(pRule, pLabel1, pLabel2);
          //if (!pNormLabelEdge->HasEnoughInstances()) // false  when has enough_instances = instances >= 1
          if(label_edge != NULL) {
            label_edge->AddLabeledNode(pNode1, pNode2);
          } else {
            std::cout << "label_edge is NULL " << std::endl;
          }
      }
    }
    else {
      /* 
        only add the closest sibling 
      */
      GraphNode* pNode2 = pNode1->ClosestGraphNode(child_nodes);
      //Label* pLabel2 = child_labels[index];
      Label* pLabel2 = pNode2->AssociatedLabel();
      LabelEdge* label_edge = GetLabelEdgeByInstance(pRule, pLabel1, pLabel2);
      if(label_edge != NULL) {
        label_edge->AddLabeledNode(pNode1, pNode2);
      } 
      else {
        std::cout << "label_edge is NULL " << std::endl;
      }
    }
  }
}

void Grammar::ComputeDerivedRuleMap(void)
{
  // indexed by predecessors
	derived_rule_map_.clear();
	for (int i=0; i<NLabels(); i++) {
		derived_rule_map_[labels_[i]] = std::vector<Rule*>();
	}
  std::cout << std::endl;
	for (int i=0; i<NRules(); i++) {
		Rule* pRule = rules_[i];
		Label* pLabel = pRule->Predecessor();
		derived_rule_map_[pLabel].push_back(pRule);
    std::cout << "rule " << i << " in derived_rule_map_ " << pRule->ToString() << std::endl;
	}
  std::cout << std::endl;
  // indexed by successors
  deriving_rule_map_.clear();
  for (int i = 0; i < NRules(); i++) {
    Rule* pRule = rules_[i];
    for (int j = 0; j < pRule->NSuccessors(); j++) {
      Label* pLabel1 = pRule->Successor(j);
      for (int k=0; k < pRule->NSuccessors(); k++) {
        Label* pLabel2 = pRule->Successor(k);
        std::pair<Label*, Label*> labelPair = std::make_pair(pLabel1, pLabel2);
        if (deriving_rule_map_.find(labelPair) == deriving_rule_map_.end()) {
            deriving_rule_map_[labelPair] = std::vector<Rule*>();
        }
        deriving_rule_map_[labelPair].push_back(pRule);
      }
    }
  }
}
void Grammar::AddKernels() {
  // Kernels for label
  std::cout << "AddLabelKernel " << std::endl << std::endl;
  for (int i = 0; i < NLabels(); i++) {
    Label* pLabel = GetLabelByIndex(i);
    pLabel->AddLabelKernel();
  }
  // Kernels for edge
  std::cout << "AddEdgeKernel " << std::endl << std::endl;
  for (int i = 0; i < NEdges(); i++) {
    LabelEdge* pLabelEdge = GetLabelEdgeByIndex(i);
    pLabelEdge->AddEdgeKernel();
  }
  std::cout << "Added all Kernel " << std::endl << std::endl;
}
void Grammar::ClearInstances(void) {
  ClearLabelInstances();
  //std::cout << "begin ClearRuleInstances " << std::endl;
  ClearRuleInstances();
  //std::cout << "begin ClearLabelEdgeInstances " << std::endl;
  ClearLabelEdgeInstances();
  std::cout << "Cleared Instances " << std::endl << std::endl;
}

void Grammar::ClearLabelInstances(void) {
  for (int i=0; i<NLabels(); i++) {
    Label* label = GetLabelByIndex(i);
    if (label == NULL) {
      std::cout << "label #" << i << " was null" << std::endl;
    } else {
		  label->ClearInstances();
    }
  }
}

void Grammar::ClearRuleInstances(void) {
  for (int i=0; i<NRules(); i++) {
    Rule* rule = GetRuleByIndex(i);
    if (rule == NULL) {
      std::cout << "Rule #" << i << " was null" << std::endl;
      std::cout << "size of rules_ " << rules_.size() << std::endl;
    } else {
	    rule->ClearInstances();
    }
  }
}

void Grammar::ClearLabelEdgeInstances(void) {
  for (int i=0; i<NEdges(); i++) {
    //GetLabelEdgeByIndex(i)->ClearInstances();
    LabelEdge* pEdge = GetLabelEdgeByIndex(i);
    pEdge->ClearInstances();
  }
}

} //ns
