#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include "json.hpp"

using json = nlohmann::json;

int test(){
  //std::string filename = "pretty.json";
  std::ifstream input("pretty.json");
  json json_graph;
  input >> json_graph;

  std::map<int, std::map<std::string, std::string> > node_blocks;
  size_t count = 0;
  for (json::iterator it = json_graph.begin(); it != json_graph.end(); ++it, count++){
    std::map<std::string, std::string> block;
    for (json::iterator itit = it->begin(); itit != it->end(); ++itit){
      std::string string_value;
      if (itit.value().is_number()){
        auto number_value = itit.value().get<float>();
        string_value = std::to_string(number_value);
      }
      else if (itit.value().is_string()){
        std::cout << "is string" << std::endl;
        string_value = itit.value().get<std::string>();
      }
      else {
        std::cout << "unknown type" << std::endl;
      }
      block.emplace(itit.key(), string_value);
    }  
    node_blocks.emplace(count, block);
  }

  for(auto const &ent1 : node_blocks) {
    auto const &outer_key = ent1.first;
    auto const &inner_map = ent1.second;
    for(auto const &ent2 : inner_map) {
      //auto const &inner_key   = ent2.first;
      
      std::string inner_key = ent2.first; 
      auto const &inner_value = ent2.second;
      std::cout <<  inner_key << "  "<< inner_value << std::endl;
      if(!strcmp(inner_key.c_str(), "children")){
        std::cout << "find children!" << std::endl;
        std::stringstream stream(inner_value);
        int n;
        while(stream >> n){
          std::cout << n << std::endl;
        }
      }
     }
  }
}
