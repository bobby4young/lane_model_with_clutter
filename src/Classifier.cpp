#include <string>
#include <vector>
#include "Classifier.h"
#include "Edge.h"

namespace lane_model {

Classifier::Classifier() {}

void Classifier::ExportTraining(LabelEdge* pEdge, std::string dirname) {
//     ofstream fout(classifierFilename.c_str());
//     // bandwith for each dimension
//     vector<double> accumDiams(Descriptor::NBinaryDistanceDims(), 0);
//     for (int i=0; i<pLabelEdge->NInstances(); i++) {
//         LabelEdgeInstance* pInstance = pLabelEdge->GetKthInstance(i);
//         const vector<double>& diam1 = pInstance->GetDiameter1();
//         const vector<double>& diam2 = pInstance->GetDiameter2();
//         if (diam1.size() != accumDiams.size()) {
//             fprintf(stderr, "NearestNeighborClassifier::ExportTraining dimension doesn't match!\n");
//             exit(-1);
//         }
//         if (diam2.size() != accumDiams.size()) {
//             fprintf(stderr, "NearestNeighborClassifier::ExportTraining dimension doesn't match!\n");
//             exit(-1);
//         }
//         for (int j=0; j<(int)diam1.size(); j++) {
//             accumDiams[j] += fmin(diam1[j], diam2[j]);
//         }
//     }
//     /*average and times a constant*/
//     if (pLabelEdge->NInstances() > 0) {
//         for (int i=0; i<(int)accumDiams.size(); i++) {
//             // average and multiply by the ratio
//             accumDiams[i] = accumDiams[i] / pLabelEdge->NInstances();
//             // minimum bandwidth
//             if (accumDiams[i] < MyConfigParser.kde_min_bandwidth)
//                 accumDiams[i] = MyConfigParser.kde_min_bandwidth;
//         }
//     }
//     // export bandwidths
//     fout<<StringUtil::Array2String(accumDiams)<<std::endl;
//     // one row for one instance
//     for (int i=0; i<pLabelEdge->NInstances(); i++) {
//         LabelEdgeInstance* pInstance = pLabelEdge->GetKthInstance(i);
//         fout<<StringUtil::Array2String(pInstance->GetPairwiseDistance())<<std::endl;
//     }
//     fout.close();
  
}

} //ns