#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include <set>
#include "Graph.h"
#include "GraphNode.h"
#include "Grammar.h"
#include "GrammarProducer.h"
#include "Label.h"
#include "json.hpp"
#include "Rule.h"
#include "ClutterProducer.h"
#include "Visualization.h"

using json = nlohmann::json;

namespace lane_model{

GrammarProducer::GrammarProducer(): raw_grammar_(NULL),
  created_grammar_(NULL) {
	raw_grammar_ = new Grammar;
  created_grammar_ = new Grammar;
}

GrammarProducer::~GrammarProducer() {
  delete raw_grammar_;
  delete created_grammar_;
  for (auto graph: graph_array_) {
    delete graph;
  }
  graph_array_.clear();
}

void GrammarProducer::ImportSceneGraphs(const std::string& dirname) {
  for (int i = 1; i < 41; i++) {
    Graph* pGraph = new Graph;
    pGraph->SetGrammar(raw_grammar_);
    std::string filename = dirname + std::to_string(i) + "labeled.json";
    std::cout << "begin import " << filename << std::endl;
    pGraph->ImportGraph(filename, i);
    graph_array_.push_back(pGraph);
		std::cout << "\033[1;33mNodes index:  \033[0m" << std::endl;		
    for (int j = 0; j < pGraph->NNodes(); j++) {
      std::cout << j << " " ;
      std::cout << pGraph->GetNodeWithIndex (j)->ToString() << std::endl;
    }
  }
}

void GrammarProducer::ImportSceneGraphsLeaveOneOut(const std::string& dirname, const int index) {
  for (int i = 1; i < 41; i++) {
    if (i == index) {
      continue;
    }
    /*
      TODO: move graph to scope of GrammarProducer, stack
      Graph graph;
      Graph* pGraph = &graph;
    */
    Graph* pGraph = new Graph;
    pGraph->SetGrammar(raw_grammar_);
    //std::string filename = dirname + "labeled.json";
    std::string filename = dirname + std::to_string(i) + "labeled.json";
    std::cout << "begin import " << filename << std::endl;
    pGraph->ImportGraph(filename, i);
    graph_array_.push_back(pGraph);
		std::cout << "\033[1;33mNodes index:  \033[0m" << std::endl;		
    for (int j = 0; j < pGraph->NNodes(); j++) {
      std::cout << j << " " ;
      std::cout << pGraph->GetNodeWithIndex(j)->ToString() << std::endl;
    }
  }
}

void GrammarProducer::CreateAndAssignGrammar(void) {
  // add label
  std::cout << "adding label " << std::endl;
  for (int i = 0; i < graph_array_.size(); i++) {
    Graph* pGraph = graph_array_[i];
    pGraph->SetGrammar(created_grammar_);
    for (int j=0; j<pGraph->NNodes(); j++) {
      GraphNode* pNode = pGraph->Node(j);
      Label* pRawLabel = pNode->AssociatedLabel();
      if (!pRawLabel) {
        std::cout << "pRawLabel in GrammarProducer is a NULL" << std::endl;  
        continue;
      }
      //switch grammar from raw_grammar_ to created_grammar_
      std::string labelName = pRawLabel->BasicCategoryName();
      Label* pCreatedLabel = created_grammar_->GetLabelByString(labelName);
      if (!pCreatedLabel) {
        pCreatedLabel = new Label(created_grammar_);
        pCreatedLabel->SetBasicCategory(labelName);
      }
      pNode->SetAssociatedLabel(pCreatedLabel);
    }
    GraphNode* root_node = pGraph->Root();
    if(created_grammar_->RootLabel() == NULL && root_node != NULL) {
      created_grammar_->SetRootLabel(root_node->AssociatedLabel());
    }
  }
  std::cout << "finish adding label " << std::endl;
  // rule set
  std::cout << "adding rule " << std::endl;
  typedef std::multimap<Label*, std::pair<std::set<Label*>, std::set<int> > > Lhs2RhsMap;
  Lhs2RhsMap lhs_2_rhs_map;
  //Lhs2RhsCardinalityMap lhs_2_rhs_cardinality_map;
  for (int i=0; i < (int)graph_array_.size(); i++) {
    Graph* pGraph = graph_array_[i];
    for (int j=0; j< pGraph->NNodes(); j++) {
      GraphNode* pNode = pGraph->Node(j);
      Label* pLabel = pNode->AssociatedLabel();
      if (!pLabel) continue;
      if (pNode->NChildren() == 0) continue;
      std::set<Label*> children_label;
      std::set<int> cardinality =  std::set<int>();
      for (int k=0; k < pNode->NChildren(); k++) {
        GraphNode* pChildNode = pNode->Child(k);
        Label* pChildLabel = pChildNode->AssociatedLabel();
        if (!pChildLabel) continue;
        // remove case where childlabel == parentlabel
        if (pChildLabel == pLabel) continue;
        children_label.insert(pChildLabel);
      }
      cardinality.insert(pNode->NChildren());

      auto lhs_2_rhs_map_it = lhs_2_rhs_map.find(pLabel);
      // if this rule doesn't exist add it
      if (lhs_2_rhs_map_it == lhs_2_rhs_map.end()) {
        lhs_2_rhs_map.insert(std::make_pair(pLabel, std::make_pair(children_label, cardinality)));
        std::cout << "rule doesn't exist  lhs_2_rhs_map size " << lhs_2_rhs_map.size() << " added with label " << pLabel->BasicCategoryName() << std::endl; 
      }
      else {
        auto rules = lhs_2_rhs_map.equal_range(pLabel);
        bool found_the_same_rule = false;
        for (Lhs2RhsMap::iterator it = rules.first; it != rules.second; ++it) {
          // check if children label are the same, if so, add cardinality, if not, add new rule
          if ( it->second.first == children_label) {
            it->second.second.insert(pNode->NChildren());
            found_the_same_rule = true;
          }
        }
        if (found_the_same_rule == false) {
          lhs_2_rhs_map.insert(std::make_pair(pLabel, std::make_pair(children_label, cardinality)));
        }
      }
    }
  }
  //initialize Rule 
  // TODO multimap
  std::map<Label*, Rule*> labels2rules;
  labels2rules.clear();
  //assert();
  for (Lhs2RhsMap::const_iterator it = lhs_2_rhs_map.begin(); it != lhs_2_rhs_map.end(); it++) {
    Label* lhs = it->first;
    const std::set<Label*>& rhs = it->second.first;
    std::vector<Label*> rhsArray;
    for (std::set<Label*>::const_iterator it = rhs.begin(); it != rhs.end(); it++) {
      rhsArray.push_back(*it);
    }
    // rhs size > 0
    if (rhsArray.size() == 0) continue;
    rhsArray.push_back(lhs);
    Rule* pRule = new Rule(created_grammar_);
    pRule->InitializeFromLabelArray(rhsArray);
    if (it->second.second.size() == 1) {
      pRule->SetSuccessorCardinalityFixed();
    }
    pRule->SetCardinality(it->second.second);
    if (created_grammar_->GetRuleByIndex(0) == NULL) {
      std::cout << "created rule is NULL !!!!!!!!!!!" << std::endl;
    }
    labels2rules[lhs] = pRule;
    //labels2rules[lhs]->SetCardinality(lhs_2_rhs_cardinality_map.find(lhs)->second);
  }
  std::cout << "finish create rule " << std::endl;
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  // add rule of duplicated input
  // int n_rules = created_grammar_->NRules();
  // for (int i = 0; i < n_rules; i++) {
  //   Rule* pRule = created_grammar_->GetRuleByIndex(i);
  //   std::vector<Label*> label_array;
  // }

  // add rules to node
  for (int i=0; i<(int)graph_array_.size(); i++) {
    Graph* pGraph = graph_array_[i];
    for (int j = 0; j < pGraph->NNodes(); j++) {
      GraphNode* pNode = pGraph->Node(j);
      //if (!pNode->IsMeaningful()) continue;
      if (pNode->IsTerminal()) {
        pNode->SetAssociatedRule(NULL);
      }
      else {
        Label* pLabel = pNode->AssociatedLabel();
        if (!pLabel) continue;
        if (labels2rules.find(pLabel) != labels2rules.end()) {
          pNode->SetAssociatedRule(labels2rules[pLabel]);
        }
        else {
          pNode->SetAssociatedRule(NULL);
        }
      }
    }
  }
  std::cout << "finish adding rule to nodes" << std::endl;
  created_grammar_->CreateEdges();
}

void GrammarProducer::ReestimateGrammarParameters(void) {
  // ClearLabelInstances();
  // ClearRuleInstances();
  // ClearLabelEdgeInstances();
	created_grammar_->ClearInstances();
  // add all new scenes
  for (int i = 0; i< (int)graph_array_.size(); i++) {
    std::cout << "begin adding " << i + 1<< " graphs to grammar " << std::endl;
    created_grammar_->AddLabeledGraph(graph_array_[i]);
    std::cout << "added " << i + 1<< " graphs to grammar " << std::endl;
	}
  std::cout << "ComputeDerivedRuleMap " << std::endl;
  created_grammar_->ComputeDerivedRuleMap();
  std::cout << "add kernel " << std::endl;
	created_grammar_->AddKernels();
	//created_grammar_->UpdateParameters();
}

void GrammarProducer::CreateLabelEdges(void) {
  // created_grammar_->CreateEdges();
  // for (int i=0; i<(int)graph_array_.size(); i++) {
  //   created_grammar_->AddLabeledGraphForLabelEdges(graph_array_[i]);
  // }
  // //CheckLabelEdgeOpenness();
}

void GrammarProducer::ExportGrammar(const std::string& dirname) {
	created_grammar_->Export(dirname);
}

Grammar* GrammarProducer::GetCreatedGrammar() {
  return created_grammar_;
}

void GrammarProducer::GenerateLabeledFile() {
  const std::string dirname = "../data/Germany/data2/";
  for (int i = 1; i < 41; i++) {
    Graph* pGraph = new Graph;
    std::string label_name = "lane";
    Label* label = new Label(raw_grammar_);
    label->SetBasicCategory(label_name);
    pGraph->SetGrammar(raw_grammar_);
    //std::string filename = dirname + "labeled.json";
    std::string filename = dirname + std::to_string(i) + ".json";
    std::cout << "begin import " << filename << std::endl;
    json json_graph = pGraph->ImportUnlabeledGraph(filename, i);
    pGraph->ProduceLabeledData(dirname, i, json_graph);
    graph_array_.push_back(pGraph);
    std::cout << "import scene graph " << i <<  " was successful" << std::endl;
  }
}

void GrammarProducer::GenerateClutter() {
  const std::string dirname = "../data/Germany/data2/";
  std::string marking_label_name = "marking_segment";
  Label* marking_label = new Label(raw_grammar_);
  marking_label->SetBasicCategory(marking_label_name);
  Label* boundary_label = new Label(raw_grammar_);
  std::string boundary_label_name = "marking_boundary";
  boundary_label->SetBasicCategory(boundary_label_name);
  for (size_t i = 31; i < 41; i ++) {
    ClutterProducer clutter;
    clutter.SetGrammar(raw_grammar_)  ;
    int clutter_type = 3;
    std::cout << "clutter begins import " << dirname << std::endl;
    clutter.ImportTerminals(dirname + std::to_string(i) + ".json");
    clutter.ProduceDataWithClutter(dirname, i, clutter_type);
  }
}

void GrammarProducer::ExportEdgeData() const {
  /*
    extract marking edge data into csv file
    describe vector for marking node edge is ego.height ego.width other.height other.width dx dy angle
  */
  std::ofstream marking_file;
  std::ofstream lane_boundary_file;
  marking_file.open ("marking.csv");
  lane_boundary_file.open ("lane_boundary.csv");
  for (int i = 0; i < graph_array_.size(); i++) {
    Graph* graph = graph_array_[i];
    graph->ExportMarkingEdgeData(marking_file);
    graph->ExportLaneBoundaryEdgeData(lane_boundary_file);
  }
  marking_file.close();
  lane_boundary_file.close();
}

} //ns
