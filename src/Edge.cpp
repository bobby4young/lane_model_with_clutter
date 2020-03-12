#include <iostream>
#include <fstream>
#include <math.h>       /* isinf, sqrt */
#include <string>
#include "Edge.h"
#include "Grammar.h"
#include "Label.h"
#include "Rule.h"
#include "Utility.h"
#include "GaussianKernel.h"
#include "json.hpp"

using json = nlohmann::json;

namespace lane_model {

LabelEdgeInstance::LabelEdgeInstance(GraphNode* pNode1, GraphNode* pNode2, const std::vector<double>& distance):
   node_1_(pNode1), node_2_(pNode2), relation_(distance) {
}

const std::vector<double>& LabelEdgeInstance::GetPairwiseDistance() const {
  return relation_;
}

LabelEdge::LabelEdge(Grammar* pGrammar): index_(-1), grammar_(NULL), rule_(NULL), 
            label_1_(NULL), label_2_(NULL), gaussian_kernel_(NULL), 
            label_name_("marking_segment") {
  if (pGrammar) {
    pGrammar->InsertLabelEdge(this);
  }
}

LabelEdge::~LabelEdge() {
  delete gaussian_kernel_;
}

void LabelEdge::ClearInstances() {
  for (int i = 0; i<edge_instances_.size(); i++) {
    delete edge_instances_[i];
  }
  edge_instances_.clear();
}

void LabelEdge::SetAssociatedRuleAndLabels(Rule* pRule, Label* pLabel_1, Label* pLabel_2) {
  if (pRule == NULL || pLabel_1 == NULL || pLabel_2 == NULL) {
    std::cout << "one of things in edge is NULL" << std::endl;
  }
  rule_ = pRule;
  label_1_ = pLabel_1;
  label_2_ = pLabel_2;
}

const std::string LabelEdge::ToString() const {
  std::string predName = "NULL";
  if (rule_) predName = rule_->Predecessor()->BasicCategoryName();
  std::string label1Name = "NULL";
  if (label_1_) label1Name = label_1_->BasicCategoryName();
  std::string label2Name = "NULL";
  if (label_2_) label2Name = label_2_->BasicCategoryName();
  
  std::string buffer = predName+"&"+label1Name+"&"+label2Name;
  return buffer;
}

Rule* LabelEdge::GetGrammarRule() const {
  return rule_;
}

Label* LabelEdge::GetLabel_1() const {
  return label_1_;
}

Label* LabelEdge::GetLabel_2() const {
  return label_2_;
}

int LabelEdge::Index() const {
  return index_;
}

int LabelEdge::NInstances() const {
  return edge_instances_.size();
} 

void LabelEdge::AddLabeledNode(GraphNode* pNode1, GraphNode* pNode2) {
  std::vector<double> relation = pNode1->Relation(pNode2);
  edge_instances_.push_back(new LabelEdgeInstance(pNode1, pNode2, relation));
}

void LabelEdge::AddEdgeKernel() {
  if (NInstances() == 0){
    utility::PrintError(ToString(), "edge_instances_ is zero");
    return;
  }
  int dimension = edge_instances_[0]->GetPairwiseDistance().size();
  realMatrixType training_data(NInstances(), dimension);
  for (int i = 1; i < NInstances(); i++) {
    std::vector<double> relation = edge_instances_[i]->GetPairwiseDistance();
    std::cout << "LabelEdge training_data added "<< i << " " << label_1_->BasicCategoryName() <<" ";
    for (auto r: relation) {
       std::cout << r << " ";
    }
    std::cout << std::endl;
    
    for ( int j = 0; j < dimension; j++) {
      training_data(i, j) = relation[j];
    }
  }
  gaussian_kernel_ = new GaussianKernel(training_data);
  // add minimum likelihood
  std::vector<double> first_relation = edge_instances_[0]->GetPairwiseDistance();
  minimum_likelihood_ = gaussian_kernel_->ComputePDF(first_relation);
  average_likelihood_ = minimum_likelihood_;
  if (NInstances() == 1) {
    return;
  }
  for (int i = 1; i < NInstances(); i++) {
    std::vector<double> relation = edge_instances_[i]->GetPairwiseDistance();
    double likelihood = gaussian_kernel_->ComputePDF(relation);
    minimum_likelihood_ = std::min(likelihood, minimum_likelihood_);
    average_likelihood_ += likelihood;
  }
  std::cout << "minimum_likelihood for edge " << label_1_->BasicCategoryName() << " is " << minimum_likelihood_ << std::endl;
  if (label_1_->BasicCategoryName() == "marking_segment") {
    minimum_likelihood_ = 1.0e-80;
  }
  if (label_1_->BasicCategoryName() == "lane_boundary") {
    minimum_likelihood_ = 1.0e-100;
  }
  minimum_likelihood_ = std::min(minimum_likelihood_, 1.0e-4);
  average_likelihood_ = average_likelihood_ / NInstances();
  std::cout << "average_likelihood for edge " << label_1_->BasicCategoryName() << " is " << average_likelihood_ << std::endl;
  std::cout << "minimum_likelihood for edge " << label_1_->BasicCategoryName() << " is " << minimum_likelihood_ << std::endl << std::endl;
  /*
    generate file for visualization
  */
  if (label_1_->BasicCategoryName() == label_name_ && false) {
    visual_relation_.resize(dimension);
    for (auto& re: visual_relation_) {
      re.resize(NInstances());
    }
    for (int i = 1; i < NInstances(); i++) {
      std::vector<double> instance_relation = edge_instances_[i]->GetPairwiseDistance();    
      for ( int j = 0; j < dimension; j++) {
        visual_relation_[j][i] = instance_relation[j];
      }
    }
    for (auto& one_dimension: visual_relation_) {
      std::sort(one_dimension.begin(), one_dimension.end());
    }
    GenerateVisualizationFile();
  }
  
}

double LabelEdge::ComputeEnergy(std::vector<double> sample) const {
  double energy = gaussian_kernel_->ComputePDF(sample);
  return energy;
  // double log_energy = -log(energy);
  // if (isinf(log_energy)) {
  //   return 1000;
  // }
  // else {
  //   return log_energy;
  // }
}

bool LabelEdge::operator==(LabelEdge pEdge) const {
  return ((GetLabel_1() == pEdge.GetLabel_1()) && (GetLabel_2() == pEdge.GetLabel_2()));
}

bool LabelEdge::IsEdge(GraphNode* pNode1, GraphNode* pNode2) const {
  std::vector<double> relation = pNode1->Relation(pNode2);
  double likelihood = ComputeEnergy(relation);
  std::cout << "node1 " << pNode1->ToString() << " " << "node2 " << pNode2->ToString();
  if (likelihood >= minimum_likelihood_) {
    std::cout  << " likelihood " << likelihood << "\033[1;32mEdge\033[0m" << std::endl;
    return true;
  }
  else {
    std::cout  << " likelihood " << likelihood << std::endl;
    return false;
  }
}

void LabelEdge::GenerateVisualizationFile() const {
  if (label_1_->BasicCategoryName() != label_name_) return;
  int index1 = 1;
  int index2 = 3;
  int dimension = visual_relation_.size();
  if (index1 >= dimension || index2 >= dimension) {
    std::cerr << "label edge " << ToString() << " relation maximum dimension is " << dimension << std::endl;
    exit(1);
  }
  int n_half_instance = NInstances() / 2;
  std::vector<double> relation;
  for (int i = 0; i < dimension; i++) {
    // relation filled with median 
    relation.push_back(visual_relation_[i][n_half_instance]);
  }
  int n_sample = 100;
  std::vector<double> parameter1;
  for (int i = 0; i < n_sample; i++) {
    double difference = (visual_relation_.at(index1).back() - visual_relation_.at(index1).front()) / n_sample;
    parameter1.push_back(visual_relation_.at(index1).front() + i * difference);
  }

  std::vector<double> parameter2;
  for (int i = 0; i < n_sample; i++) {
    double difference = (visual_relation_.at(index2).back() - visual_relation_.at(index2).front()) / n_sample;
    parameter2.push_back(visual_relation_.at(index2).front() + i * difference);
  }
  json json_file;
  for(int i = 0; i < n_sample; i++) {
    for(int j = 0; j < n_sample; j++) {
      // std::vector<double> local_relation{relation1[i], relation2[j],  median[2], median[3], median[4]};
      // double likelihood = gaussian_kernel_->ComputePDF(local_relation);
      // json_file[i][j] = {relation1[i], relation2[j], likelihood};
      // if (likelihood > 0.2)
      // std::cout << "json file data " << relation1[i] <<" " << relation2[j] << " " << likelihood << std::endl;
      relation[index1] = parameter1[i];
      relation[index2] = parameter2[j];
      double likelihood = gaussian_kernel_->ComputePDF(relation);
      json_file[i][j] = {parameter1[i], parameter2[j], likelihood};
    }
  }
  std::ofstream o1("visualization_" + label_name_ +std::to_string(index1) + std::to_string(index2) + ".json");
  std::ofstream o2("visualization.json");
  o1 << std::setw(6) << json_file << std::endl;
  o2 << std::setw(6) << json_file << std::endl;
}

} //ns
