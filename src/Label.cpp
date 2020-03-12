#include <iostream>
#include <math.h>       /* isinf, sqrt */
#include "Label.h"
#include "Grammar.h"
#include "GaussianKernel.h"

namespace lane_model{
  
LabelInstance::LabelInstance(GraphNode* pNode):
  node_(pNode), associated_node_index_(-1) {
	associated_node_index_ = pNode->GraphIndex();
}

const std::vector<double> LabelInstance::GetValue() const {
  //std::cout << "GetValue " << node_->GraphIndex() << std::endl;
  return node_->Value();
}

Label::Label(Grammar* pGrammar): grammar_(NULL), index_(-1), is_terminal_(false) {
	if (pGrammar != NULL) pGrammar->InsertLabel(this);
}

void Label::SetBasicCategory(const std::string& category) {
  basic_category_ = category;
}

std::string Label::BasicCategoryName() const {
  return basic_category_;
}

void Label::ClearInstances() {
	for (int i=0; i<(int)label_instances_.size(); i++) {
		delete label_instances_[i];
	}
	label_instances_.clear();
}

void Label::AddLabeledNode(GraphNode* pNode) {
	LabelInstance* pLabelInstance = new LabelInstance(pNode);
	label_instances_.push_back(pLabelInstance);
}

Grammar* Label::GetGrammar() const {
  return grammar_;
}

int Label::Index() const {
	return index_;
}

int Label::NInstances() const {
	return label_instances_.size();
}

void Label::AddLabelKernel() {
  std::cout  << std::endl << "Label AddLabelKernel "<< BasicCategoryName() << std::endl;
  if (NInstances() == 0) {
    std::cout << "label_instances_ is zero" << std::endl;
    return;
  }
  int dimension = label_instances_[0]->GetValue().size();
  std::cout << "dimension "<< dimension << std::endl;
  realMatrixType training_data(NInstances(), dimension);
  for (int i = 0; i < NInstances(); i++) {
    std::vector<double> value = label_instances_[i]->GetValue();
    std::cout << "Label training_data added "<< i << " " << BasicCategoryName() <<" ";
    for (auto v: value) {
      std::cout << v << " ";
    }
    std::cout << std::endl;
    for ( int j = 0; j < dimension; j++) {
      training_data(i, j) = value[j];
    }
  }
  gaussian_kernel_ = new GaussianKernel(training_data);
  
  // add minimum likelihood
  std::vector<double> first_value = label_instances_.front()->GetValue();
  minimum_likelihood_ = gaussian_kernel_->ComputePDF(first_value);
  average_likelihood_ = minimum_likelihood_;
  if (NInstances() == 1) {
    return;
  }
  for (int i = 1; i < NInstances(); i++) {
    std::vector<double> value = label_instances_[i]->GetValue();
    double likelihood = gaussian_kernel_->ComputePDF(value);
    minimum_likelihood_ = std::min(likelihood, minimum_likelihood_);
    average_likelihood_ += likelihood;
  }
  average_likelihood_ = average_likelihood_ / NInstances();
  std::cout << "minimum_likelihood for label " << BasicCategoryName() << " was " << minimum_likelihood_ << std::endl;
  //minimum_likelihood_ = std::min(minimum_likelihood_, 1.0e-50);
  std::cout << "minimum_likelihood for label " << BasicCategoryName() << " is " << minimum_likelihood_ << std::endl;
  std::cout << "average_likelihood_ for label " << BasicCategoryName() << " is " << average_likelihood_ << std::endl;
}

double Label::ComputeEnergy(std::vector<double> sample) const {
  double energy = gaussian_kernel_->ComputePDF(sample);
  return energy;
  // double log_energy = -log(energy);
  // if (isinf(log_energy)) {
  //   return 1e20;
  // }
  // else {
  //   return log_energy;
  // }
}

bool Label::IsLabel(GraphNode* pNode) const {
  //if (ComputeEnergy(pNode->Value()) >= minimum_likelihood_) {
  if (ComputeEnergy(pNode->Value()) >= std::min(minimum_likelihood_, 1.0e-50)) {
    return true;
  }
  else {
    return false;
  }
}
bool Label::IsMinimalLabel(GraphNode* pNode) const {
  if (ComputeEnergy(pNode->Value()) >= minimum_likelihood_) {
    return true;
  }
  else {
    return false;
  }
}

} //ns