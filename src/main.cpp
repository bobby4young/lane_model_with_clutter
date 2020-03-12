#include <chrono> 
#include <string>
#include <iostream>
#include <fstream>
#include "GrammarProducer.h"
#include "Parser.h"

using namespace lane_model;

int main(int argc, char **argv) {

  std::string dirname = "../data/Germany/data2/";

  // GrammarProducer producer;
  // producer.ImportSceneGraphs(dirname);
  // producer.CreateAndAssignGrammar();
  // producer.ReestimateGrammarParameters();
  // producer.ExportGrammar("../grammar/");
  // producer.GenerateLabeledFile();
  // producer.GenerateClutter();
  // if (argc == 2) {
  //   // which folder in unparsed/ as input for parser
  //   std::string folder_number(argv[1]);
  //   // Maybe parser should be initialized with grammar
  //   Parser parser;
  //   parser.SetGrammar(producer.GetCreatedGrammar());
  //   //std::string filename = "../data/unparsed/input.json";
  //   //parser.Import(folder_number);
  //   parser.ImportTerminals(folder_number);
  //   //parser.CombinationParse2();
  //   parser.BottomUpParse();
  //   //parser.Parse();
  //   // parser.ComputeGraphsEnergy();
  //   // parser.PickBestGraph();
  //   //parser.Export(folder_number);
  // }
  // else {
  //   std::cout << std::endl << "please give the number of directory to unparsed file, " << std::endl
  //     << "e.g. for unparsed/1/input.json just 1 " << std::endl<< std::endl;
  // }

  // int i = std::stoi(argv[1]);
  // GrammarProducer producer;
  // //producer.ImportSceneGraphs(dirname);
  // producer.ImportSceneGraphsLeaveOneOut(dirname, i);
  // producer.CreateAndAssignGrammar();
  // producer.ReestimateGrammarParameters();
  // producer.ExportGrammar("../grammar/");

  // Parser parser;
  // int clutter_type = 0;
  // auto start = std::chrono::high_resolution_clock::now();
  // parser.SetGrammar(producer.GetCreatedGrammar());
  // parser.ImportTerminals(std::to_string(i), clutter_type);
  // parser.BottomUpParse();
  // parser.ComputeGraphsEnergy();
  // parser.PickBestGraph(clutter_type);
  // auto stop = std::chrono::high_resolution_clock::now(); 
  // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  // parser.Log(duration, i, clutter_type);
  
  /*
    leave one out cross validation
  */
  // for (size_t i = 1; i < 9; i++) {
  //   GrammarProducer producer;
  //   producer.ImportSceneGraphsLeaveOneOut(dirname, i);
  //   producer.CreateAndAssignGrammar();
  //   producer.ReestimateGrammarParameters();
  //   producer.ExportGrammar("../grammar/");

  //   Parser parser;
  //   int clutter_type = 2;
  //   auto start = std::chrono::high_resolution_clock::now();
  //   parser.SetGrammar(producer.GetCreatedGrammar());
  //   parser.SetVisual(false);
  //   parser.ImportTerminals(std::to_string(i), clutter_type);
  //   parser.BottomUpParse();
  //   parser.ComputeGraphsEnergy();
  //   parser.PickBestGraph(clutter_type);
  //   auto stop = std::chrono::high_resolution_clock::now(); 
  //   auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  //   parser.Log(duration, i, clutter_type);
  // }

  /*

  */
  GrammarProducer producer;
  producer.ImportSceneGraphs(dirname);
  //producer.ImportSceneGraphsLeaveOneOut(dirname, i);
  producer.CreateAndAssignGrammar();
  producer.ReestimateGrammarParameters();
  producer.ExportEdgeData();
  return 0;
}
