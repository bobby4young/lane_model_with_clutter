#ifndef LANE_MODEL_CLASSIFIER_H_
#define LANE_MODEL_CLASSIFIER_H_

namespace lane_model {

class LabelEdge;

class Classifier {
 public:
  Classifier(/* args */);
  ~Classifier();
  void ExportTraining(LabelEdge* pEdge, std::string dirname);
 private:

};

}


#endif // LANE_MODEL_CLASSIFIER_H_