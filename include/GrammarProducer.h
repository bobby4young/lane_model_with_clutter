#ifndef LANE_MODEL_GRAMMARPRODUCER_H_
#define LANE_MODEL_GRAMMARPRODUCER_H_

#include <vector>

namespace lane_model {

class Graph;
class Grammar;

class GrammarProducer {
 public:
  GrammarProducer();
  ~GrammarProducer();
  void ImportSceneGraphs(const std::string& dirname);
  void ImportSceneGraphsLeaveOneOut(const std::string& dirname, const int index);
  void ExportGrammar(const std::string& dirname);
  void CreateAndAssignGrammar(void);
  void ReestimateGrammarParameters(void);
  void CreateLabelEdges(void);
  Grammar* GetCreatedGrammar();
  // Generate unreliable labeled data, needs manual check afterwards
  void GenerateLabeledFile();
  void GenerateClutter();
  void ExportEdgeData() const;
 private:
  Grammar* raw_grammar_;
  Grammar* created_grammar_;
  // array of labeled data
	std::vector<Graph*> graph_array_;
};

} // ns
#endif //LANE_MODEL_GRAMMARPRODUCER_H_
