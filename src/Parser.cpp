#include <algorithm> 
#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include "Parser.h"
#include "json.hpp"
#include "Grammar.h"
#include "Graph.h"
#include "GraphNode.h"
#include "Label.h"
#include "Edge.h"
#include "Rule.h"
#include "Utility.h"
#include "Visualization.h"

#include <thread>         // std::thread

#include <iomanip>
#include <unordered_set>
#include <set>
#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using json = nlohmann::json;

namespace lane_model {

Parser::Parser(): is_visual_(true){}

Parser::~Parser() {
  for (auto graph: candidate_graphs_) {
    delete graph;
  }
  //std::cout << "parser destructor end" << std::endl;
}

void Parser::SetGrammar(Grammar* pGrammar) {
  if(!pGrammar) {
    std::cout << "grammar passed to parser is NULL" << std::endl;
    return;
  }
  grammar_ = pGrammar;
  root_label_= pGrammar->RootLabel();
}

void Parser::ImportTerminals(std::string folder_number, const int clutter_type) {
  std::cout << "Parser began import " << folder_number << std::endl;
  std::string filename;
  switch(clutter_type) {
    case 0:
      filename = "../data/Germany/data2/" + folder_number + ".json";
      break;
    case 1:
      filename = "../data/Germany/data2/" + folder_number + "clutter.json";
      break;
    case 2:
      filename = "../data/Germany/data2/" + folder_number + "clutter2.json";
      break;
    case 3:
      filename = "../data/Germany/data2/" + folder_number + "clutter3.json";
      break;
    case 4:
      filename = "../data/Germany/data2/" + folder_number + "clutter4.json";
      break;
    default:  
      utility::PrintError("clutter type error");
      break;
  }
  graph_.SetGraphIndex(std::stoi(folder_number));
  base_graph_.SetGraphIndex(std::stoi(folder_number));
  graph_.SetGrammar(grammar_);

  std::ifstream input(filename);
  json json_graph;
  input >> json_graph;

  // data unit is not in meters 
  bool change_scale = false;
  static int scale;
  std::vector<double> array_x, array_y;
  for (json::iterator it = json_graph.begin(); it != json_graph.end(); ++it) {
    if (utility::IsStringANumber(it.key())) {
      for (json::iterator itit = it->begin(); itit != it->end(); ++itit) { 
        if(!itit.key().compare("center_x")) {
          array_x.push_back(itit.value().get<double>());
        }
        else if(!itit.key().compare("center_y")) {
          array_y.push_back(itit.value().get<double>());
        }
      }
    }
    else if (!it.key().compare(0, 14, "boundary_solid")) {
      for (json::iterator itit = it->begin(); itit != it->end(); ++itit) {
    		if(!itit.key().compare("points")) {
    		  for (json::iterator point_it = itit->begin(); point_it != itit->end(); ++point_it) {
    		    for (json::iterator coordinate_it = point_it->begin(); coordinate_it != point_it->end(); ++coordinate_it) {
    		      if(!coordinate_it.key().compare("x")) {
    		        array_x.push_back(coordinate_it.value().get<double>());
    		      }
    		      else if(!coordinate_it.key().compare("y")) {
    		        array_y.push_back(coordinate_it.value().get<double>());
    		      }
    		    }
    		  }	
    		}			
			}
    }
    else if (!it.key().compare(0, 5, "scale")) {
			change_scale = true;
      scale = it.value().get<int>();
    }
  }
  double offset_x = *std::min_element(array_x.begin(), array_x.end()) - 10; 
  double offset_y = *std::min_element(array_y.begin(), array_y.end()) - 10;
  double scene_width = *std::max_element(array_x.begin(), array_x.end()) - offset_x;
  double scene_height = *std::max_element(array_y.begin(), array_y.end()) - offset_y;
  graph_.SetSceneSize(std::max(scene_width, scene_height));

  // std::cout << "offset_x " << offset_x << " offset_y " << offset_y << std::endl; 
  std::map<int, std::map<std::string, std::string> > node_blocks;
  size_t count = 0;
  for (json::iterator it = json_graph.begin(); it != json_graph.end(); ++it, count++) {
		int id;   
    if (utility::IsStringANumber(it.key())) {
      // key of markings is a number. This is a marking node
			cv::Point2f center;
    	cv::Size2f size;
    	float angle;
    	Label* label = NULL;
			for (json::iterator itit = it->begin(); itit != it->end(); ++itit) {
    	  if(!itit.key().compare("center_x")) {
    	    center.x = itit.value().get<double>() - offset_x;
    	    // std::cout << "center.x  " << center.x << std::endl; 
    	  }
    	  else if(!itit.key().compare("center_y")) {
    	    center.y = itit.value().get<double>() - offset_y;
    	    // std::cout << "center.y  " << center.y << std::endl; 
    	  }
    	  else if(!itit.key().compare("height") || !itit.key().compare("length")) {
    	    size.height = itit.value().get<double>();
    	    // std::cout << "size.height  " << size.height << std::endl; 
    	  }
    	  else if(!itit.key().compare("width")) {
    	    size.width = itit.value().get<double>();
    	  }
    	  else if(!itit.key().compare("orientation")) {
    	    angle = itit.value().get<double>();
    	  }
		 	  else if(!itit.key().compare("id")) {
    	    id = itit.value().get<int>();
    	  }
				else if(!itit.key().compare("label")) {
    		  std::string label_name = itit.value().get<std::string>();
    		  label = grammar_->GetLabelByString(label_name);
				}
			}
    	if (size.width > size.height) {
    	  // std::cout << "height width angle before" << size.height << " " << size.width <<" "<< angle << std::endl;
    	  float width= size.height;
    	  size.height = size.width;
    	  size.width = width;
    	  angle += 90.0;
    	}
    	// std::cout << "height width angle after" << size.height << " " << size.width <<" "<< angle << std::endl;
    	// center.x *= 20;
    	// center.y *= 20;
    	// size.width *= 20;
    	// size.height *= 20;

    	cv::RotatedRect cv_rect(center, size, angle);
			MarkingNode* marking_node = new MarkingNode(&graph_, id);
			marking_node->SetBox(cv_rect);
			marking_node->SetAssociatedLabel(label);
    	Label* pre_label = grammar_->GetLabelByString("marking_boundary");
      Rule* rule = grammar_->GetRulesDerivedFromLabel(pre_label).front();
      marking_node->SetAssociatedRule(rule);
      markings_.push_back(marking_node);
    }
		else if (!it.key().compare(0, 14, "boundary_solid")) {
      // solid line node
			std::vector<Point> points;
    	Label* label = NULL;
			for (json::iterator itit = it->begin(); itit != it->end(); ++itit) {
    		if(!itit.key().compare("points")) {
    		  for (json::iterator point_it = itit->begin(); point_it != itit->end(); ++point_it) {
    		    double x, y;
    		    for (json::iterator coordinate_it = point_it->begin(); coordinate_it != point_it->end(); ++coordinate_it) {
    		      if(!coordinate_it.key().compare("x")) {
    		        x = coordinate_it.value().get<double>() - offset_x;
    		      }
    		      else if(!coordinate_it.key().compare("y")) {
    		        y = coordinate_it.value().get<double>() - offset_y;
    		      }
    		    }
    		    points.emplace_back(x, y);
    		  }	
    		}
				else if(!itit.key().compare("id")) {
    	    id = graph_.NNodes();
    	  }
				else if(!itit.key().compare("label")) {
    		  //std::string label_name = itit.value().get<std::string>();
    		  std::string label_name = "marking_boundary";
    		  label = grammar_->GetLabelByString(label_name);
    		  if (!label) {
            std::cout << "Unknown label: " << label_name << " in Parser" << std::endl;
            exit(1);
    		  }
				}
			}
			Polyline polyline(points);
			
			BoundaryNode* boundary_node = new BoundaryNode(&graph_, id, polyline);
			boundary_node->SetAssociatedLabel(label);
      Label* pre_label = grammar_->GetLabelByString("lane");
      Rule* rule = grammar_->GetRulesDerivedFromLabel(pre_label).front();
      boundary_node->SetAssociatedRule(rule);
      boundaries_.push_back(boundary_node);
			graph_.AddGraphBoundary(polyline);
    }
  }
  n_leaves_node_ = graph_.NNodes();
  base_graph_ = graph_;
  std::cout << "Parser imported a graph " << filename << std::endl;
}

void Parser::ComputeGraphsEnergy() {
  for (std::vector<Graph*>::iterator it = candidate_graphs_.begin(); it != candidate_graphs_.end(); it++) {
    std::vector<SceneNode*> scenes = (*it)->GetScenes();
    scenes_.insert(scenes_.end(), scenes.begin(), scenes.end());
  }
  utility::PrintGreen("scenes_ size ", scenes_.size());
  if (scenes_.size() == 0) {
    return;
  }
  energy_array_.resize(scenes_.size());
  int index = 0;
  for (std::vector<SceneNode*>::iterator it = scenes_.begin(); it != scenes_.end(); it++) {
    int n_nodes = 1;
    // add number of roads
    n_nodes += ((*it)->Children().size());
    for (auto road: (*it)->Children()) {
      // add number of lanes
      n_nodes += (road->Children().size());
    }
    // add number of boundaries
    n_nodes += (*it)->MyGraph()->NBoundaries();
    //energy_array_[it - scenes_.begin()] = ComputeGraphEnergy(*it);
    //double energy = std::pow(ComputeGraphEnergy(*it), 1.0 / ((*it)->MyGraph()->NNodes()));
    double energy = std::pow(ComputeGraphEnergy(*it), 1.0 / n_nodes);
    energy_array_[it - scenes_.begin()] = energy;
    std::cout << " graph index " << (*it)->MyGraph()->ToString();
    std::cout << " node index " << (*it)->ToString();
    std::cout << " energy " << energy_array_[it - scenes_.begin()] << std::endl;
    for (auto road: (*it)->Children()) {
      std::cout << (*it)->ToString() << " road: " << std::setw(20) <<  road->ToString() << " lane: " ;
      for (auto lane: road->Children()) {
        std::cout << lane->ToString() << " ";
      }
      std::cout << std::endl;
    }
    index ++;
  }

}

double Parser::ComputeGraphEnergy(GraphNode* graph_node) {
  double energy = 1.0;
  ComputeRuleEnergyRecursive(graph_node, energy);
  return energy;
}

void Parser::ComputeRuleEnergyRecursive(GraphNode* pGraphNode, double& energy) {
  if (pGraphNode->IsTerminal()) {
    return;
  }
  else {
    for (int i = 0; i < pGraphNode->NChildren(); i++) {
      GraphNode* pChild_node = pGraphNode->Child(i);
      ComputeRuleEnergyRecursive(pChild_node, energy);
    }
  }
  double edge_energy = 1.0;
  // for (int i = 0; i < pGraphNode->NChildren(); i++) {
  //   GraphNode* pChild_node_1 = pGraphNode->Child(i);
  //   Rule* pRule = pGraphNode->AssociatedRule();
  //   Label* pLabel_1 = pChild_node_1->AssociatedLabel();
  //   //double one_edge_energy = 1;
  //   for (int j = 0; j < pGraphNode->NChildren(); j++) {
  //     if (i == j) continue;
  //     GraphNode* pChild_node_2 = pGraphNode->Child(j);
  //     Label* pLabel_2 = pChild_node_2->AssociatedLabel();
  //     LabelEdge* pLabel_edge = grammar_->GetLabelEdgeByInstance(pRule, pLabel_1, pLabel_2);
  //     if (pLabel_edge == NULL) {
  //       std::cout << "label edge is NULL" << std::endl;
  //       continue;
  //     }
  //     std::vector<double> relation = pChild_node_1->Relation(pChild_node_2);
  //     edge_energy *= pLabel_edge->ComputeEnergy(relation);
  //   }
  // }
  Label* pLabel = pGraphNode->AssociatedLabel();
  //if (pLabel != grammar_->RootLabel()) {

  double label_energy = 1.0;
  if (pLabel->BasicCategoryName() != "marking_segment" && pLabel->BasicCategoryName() != "lane_boundary") {
    label_energy = pLabel->ComputeEnergy(pGraphNode->Value());
  }
  energy *= label_energy;
  return;

  if (pGraphNode->NChildren() == 1) {
    edge_energy = label_energy;
    //energy += edge_energy;
  }
  else {
    //edge_energy = edge_energy / (pow(pGraphNode->NChildren(), 2) - pGraphNode->NChildren());
    edge_energy = pow(edge_energy, 1.0 / (pow(pGraphNode->NChildren(), 2) - pGraphNode->NChildren()));
  }
  
  //energy += edge_energy;

  if (pLabel->BasicCategoryName() != "road") {
    //energy *= label_energy * edge_energy;
    energy *= label_energy * edge_energy;
  }
  else {
    energy *= edge_energy;
    //energy += edge_energy;
    //energy += energy / pGraphNode->NChildren();
  }
}

void Parser::PickBestGraph(const int clutter_type) {
  if (energy_array_.size() == 0) {
    return;
  }
  int max_index = std::max_element(energy_array_.begin(), energy_array_.end()) - energy_array_.begin();
  //  int min_index = std::distance(energy_array_.begin(), std::min_element(energy_array_.begin(), energy_array_.end()));
  std::cout << std::endl << "Best energy: " <<  *std::max_element(energy_array_.begin(), energy_array_.end()) 
    << " Best graph: " << scenes_[max_index]->MyGraph()->ToString() 
    << " Best node: " <<  scenes_[max_index]->ToString() << " likelihood " << energy_array_[max_index] << std::endl;
  scenes_[max_index]->AddSuccessorsInLanes();
  if (is_visual_) {
    Visualization visualizer(&graph_);
    cv::Mat marking_image = visualizer.DrawMarkings();
    cv::Mat lane_image = visualizer.DrawLanes(scenes_[max_index]);
    cv::Mat lane_boundary_image = visualizer.DrawLaneBoundary(scenes_[max_index]->MyGraph());
    cv::Mat boundary_image = visualizer.DrawBoundary(scenes_[max_index]->MyGraph());
    //general_boundary_image is the image contains all possible boundaries, including those not necessarrily in the final graph
    cv::Mat general_boundary_image = visualizer.DrawBoundary(&graph_);
    switch(clutter_type) {
      case 0: {
        imwrite(std::to_string(graph_.GraphIndex()) + "_no_clutter_markings.png", marking_image);
        imwrite(std::to_string(graph_.GraphIndex()) + "_no_clutter_boundary.png", boundary_image);
        imwrite(std::to_string(graph_.GraphIndex()) + "_no_clutter_lane_boundary.png", lane_boundary_image);
        imwrite(std::to_string(graph_.GraphIndex()) + "_no_clutter_general_boundary.png", general_boundary_image);
        imwrite(std::to_string(graph_.GraphIndex()) + "_no_clutter_lane.png", lane_image);
        break;
      }
      case 1: {
	      imwrite(std::to_string(graph_.GraphIndex()) + "_clutter_markings.png", marking_image);
	      imwrite(std::to_string(graph_.GraphIndex()) + "_clutter_boundary.png", boundary_image);
	      imwrite(std::to_string(graph_.GraphIndex()) + "_clutter_general_boundary.png", general_boundary_image);
	      imwrite(std::to_string(graph_.GraphIndex()) + "_clutter_lane_boundary.png", lane_boundary_image);
	      imwrite(std::to_string(graph_.GraphIndex()) + "_clutter_lane.png", lane_image);
        break;
      }
      case 2: {
	      imwrite(std::to_string(graph_.GraphIndex()) + "_clutter2_markings.png", marking_image);
	      imwrite(std::to_string(graph_.GraphIndex()) + "_clutter2_boundary.png", boundary_image);
	      imwrite(std::to_string(graph_.GraphIndex()) + "_clutter2_general_boundary.png", general_boundary_image);
	      imwrite(std::to_string(graph_.GraphIndex()) + "_clutter2_lane_boundary.png", lane_boundary_image);
	      imwrite(std::to_string(graph_.GraphIndex()) + "_clutter2_lane.png", lane_image);
        break;
      }
      case 3: {
	      imwrite(std::to_string(graph_.GraphIndex()) + "_clutter3_best_graph_markings.png", marking_image);
	      imwrite(std::to_string(graph_.GraphIndex()) + "_clutter3_general_boundary.png", general_boundary_image);
	      imwrite(std::to_string(graph_.GraphIndex()) + "_clutter3_best_graph_boundary.png", boundary_image);
	      imwrite(std::to_string(graph_.GraphIndex()) + "_clutter3_best_graph_lane_boundary.png", lane_boundary_image);
	      imwrite(std::to_string(graph_.GraphIndex()) + "_clutter3_best_graph_lane.png", lane_image);
        break;
      }
      case 4: {
	      imwrite(std::to_string(graph_.GraphIndex()) + "_clutter4_best_graph_markings.png", marking_image);
	      imwrite(std::to_string(graph_.GraphIndex()) + "_clutter4_general_boundary.png", general_boundary_image);
	      imwrite(std::to_string(graph_.GraphIndex()) + "_clutter4_best_graph_boundary.png", boundary_image);
	      imwrite(std::to_string(graph_.GraphIndex()) + "_clutter4_best_graph_lane_boundary.png", lane_boundary_image);
	      imwrite(std::to_string(graph_.GraphIndex()) + "_clutter4_best_graph_lane.png", lane_image);
        break;
      }
      default:  
        utility::PrintError("clutter type error");
        break;
    }
  }
}

void Parser::PrintGraph(GraphNode* node) const {
  if (node->NChildren() == 0) {
    return;
  }
  std::cout << "parent: " << std::setw(20) <<  node->ToString() << " children: " ;
  for (int i = 0; i < node->NChildren(); i++) {
    GraphNode* child_node = node->Child(i);
    std::cout << std::setw(15) << std::left << child_node->ToString() << " ";
  }
  //std::cout << " Label energy: " << node->AssociatedLabel()->ComputeEnergy(node->GetDescriptor()->Value());
  std::cout << std::endl;
  for (int i = 0; i < node->NChildren(); i++) {
    GraphNode* child_node = node->Child(i);
    PrintGraph(child_node);
  }
}

void Parser::CombinationParse() {
/*
  1. get terminals, generate rules(including different cardinality)
  2. parse  
*/
  int node_index = graph_.NNodes();
  std::cout << "begin Combination Parse " << std::endl;
  std::multimap<std::string, std::vector<std::string> > boundary_rules;
  std::multimap<std::string, std::vector<std::string> > lane_boundary_rules;
  std::multimap<std::string, std::vector<std::string> > lane_rules;
  std::multimap<std::string, std::vector<std::string> > road_rules;
  std::multimap<std::string, std::vector<std::string> > scene_rules;

  int n_markings = n_leaves_node_ - 2;
  if (n_markings < 1) {
    std::cout << "number of markings must larger than zero" << std::endl;
  }
  /*********************************************************************************************
    markings layer. add edge/ sibling between markings
  *********************************************************************************************/
  std::cout << "\033[1;33mFirst layer\033[0m" << " markings" << std::endl;
  for (std::vector<MarkingNode*>::iterator it = markings_.begin(); it != markings_.end(); it++) {
    MarkingNode* pnode = *it;
    for (std::vector<MarkingNode*>::iterator itit = it + 1; itit != markings_.end(); itit++) {
      GraphNode* sibling = dynamic_cast<GraphNode*>(*itit);
      pnode->AddSibling(sibling);
    }
  }
  for (std::vector<MarkingNode*>::iterator it = markings_.begin(); it != markings_.end(); it++) {
    MarkingNode* pnode = *it;
    pnode->PrintSibling();
  }
  std::cout << std::endl;
  /*********************************************************************************************
    boundaries layer. Breadth-first search add boundary
  *********************************************************************************************/
  std::cout << "\033[1;33mSecond layer\033[0m" << " boundaries" << std::endl;
  std::vector<BoundaryNode*> marking_boundaries = BFSAddBoundary();
  boundaries_.insert(boundaries_.end(), marking_boundaries.begin(), marking_boundaries.end());

  std::cout << "number of parser boundaries_ " << boundaries_.size() << std::endl;
  for (auto boundary: boundaries_) {
    utility::PrintWarning("boundary", boundary->ToString());
    for (auto child: boundary->Children()) {
      std::cout << child->ToString();
    }
    std::cout << std::endl;
  }
  //exit(0);
  Label* boundary_label = grammar_->GetLabelByString("marking_boundary");
  int deleted_boundary = 0;
  for (std::vector<BoundaryNode*>::iterator it = boundaries_.begin(); it != boundaries_.end(); it++) {
    BoundaryNode* b = *it;
    utility::PrintWarning("new boudnary node", b->ToString());
    bool is_label = boundary_label->IsLabel(*it);
    std::cout << b->ToString() << " ";
    for (auto m: b->Children()) {
      std::cout <<  " " << m->ToString();
    }
    std::cout << std::endl;
    if (is_label) {
      std::cout << " likelihood " << boundary_label->ComputeEnergy(b->Value()) << " " << is_label << "\033[1;31mLabel\033[0m" << std::endl;
    }
    else {
      std::cout << " likelihood " << boundary_label->ComputeEnergy(b->Value()) << " " << is_label << std::endl;
    }
    // for (auto v: b->Value()) {
    //   std::cout <<  " " << v;
    // }
    if (is_label) {
      //good_boundary++;
    }
    else {
      deleted_boundary++;
      boundaries_.erase(it);
      graph_.EraseBoundaryNode(b);
      it--;
    }
  }
  std::cout << "good_boundary" << boundaries_.size() << " deleted boundaries " << deleted_boundary << std::endl;
  std::cout << std::endl;

  /*********************************************************************************************
    lane boundaries layer. counter generates lane boundaries
  *********************************************************************************************/
  std::cout << "\033[1;33mThird layer\033[0m" << " lane boundaries" << std::endl;
  graph_.GenerateLaneBoundary();
  lane_boundaries_ = graph_.GetLaneBoundaries();
  // TODO add siblings in lane boundaries
  for (auto lane_boundary1: lane_boundaries_) {
    for (auto lane_boundary2: lane_boundaries_) {
      if (lane_boundary1->AddSibling(lane_boundary2)) {
        std::cout << lane_boundary1->ToString() << " and " << lane_boundary2->ToString() << " are siblings" << std::endl;    
      }
    }
  }

  Visualization visualizer(&graph_);
  for (auto lane_boundary: lane_boundaries_) {
    std::cout << lane_boundary->ToString() << " " << lane_boundary->Child(0)->ToString() << std::endl;
    Polyline polyline = lane_boundary->GetPolyline();
    //visualizer.DrawPolylinesWithColor(polyline, i);
  }
  // for (int i = 2; i < 6; i++) {
  //   std::vector<std::string> children(i + 1, "boundary");
  //   lane_boundary_rules.emplace("scene", children);
  //   std::cout << "new rule scene -> " << i + 1  << " boundary" << std::endl;
  // }
  // int n_possibility = 0;
  // for (auto rule: lane_boundary_rules) {
  //   // we have one rule with fix cardinality, so find the number of boundaries and insert as children
  //   int n_children = rule.second.size();
  //   std::vector<std::vector<int> > combi = utility::Combination(good_boundary, n_children);
  //   int n_boundary = combi.size();
  //   n_possibility += n_boundary;
  // }
  // std::cout << "n possibilities " << n_possibility << std::endl;

  //exit(5);
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  std::cout << "n lane boundaries " << lane_boundaries_.size() << std::endl;

  
  /*********************************************************************************************
    lane  layer. combination generates lane 
  *********************************************************************************************/
  std::cout << "\033[1;33mFourth layer: Lanes\033[0m" << std::endl;
  std::vector<std::string> children(2, "lane_boundary");
  lane_rules.emplace("lane", children);
  std::cout << "new rule lane -> lane_boundary lane_boundary" << std::endl;
  // we have one rule with fix cardinality
  int two_boundary = 2;
  //int n_boundary = combination(boundaries_.size(), n_children);
  //std::cout << "combination boundary -> " << n_boundary << " boundary" << std::endl;
  //std::cout << "n_children " << n_children << std::endl;
  std::vector<std::vector<int> > combi = utility::Combination(lane_boundaries_.size(), two_boundary);
  int n_lane = combi.size();
  for (int i = 0; i < n_lane; i ++) {
    // each loop is a boundary
    int child_index_1 = combi[i][0] - 1;
    int child_index_2 = combi[i][1] - 1;
    LaneBoundaryNode* child1 = lane_boundaries_[child_index_1];
    LaneBoundaryNode* child2 = lane_boundaries_[child_index_2];
    if (child1->IsSibling(child2)) {
      // if those two are siblings
      LaneNode* pnode = new LaneNode(&graph_, graph_.NNodes());
      Label* label = grammar_->GetLabelByString("lane");
      pnode->SetAssociatedLabel(label);
      Label* pre_label = grammar_->GetLabelByString("road");
      Rule* rule = grammar_->GetRulesDerivedFromLabel(pre_label).front();
      pnode->SetAssociatedRule(rule);      
      pnode->AddBoundary(child1);
      pnode->AddBoundary(child2);
      for (auto lane: lanes_) {
        pnode->AddSibling(lane);
      }
      lanes_.push_back(pnode);
      node_index++;
    }
    else {
      //std::cout << child1->ToString() <<" and " << child2->ToString() << " are not siblings!!!" << std::endl;
    }
  }
  std::cout << "number of lanes " << lanes_.size() << std::endl;
  //for (std::vector<Node*>::iterator lane_it = lanes_.begin(); lane_it != lanes_.end(); lane_it++) {
  // int n_lanes = std::min(4, markings_.size()); = lanes_.size();
  // for (int i = 0; i < n_lanes = std::min(4, markings_.size());; i++) {
  //   if (lanes_[i]->Children().size() != 2) {
  //     //std::cout << "begin found one wrong lane " << i << " children " << lanes_[i]->Children().size() << " n_lanes = std::min(4, markings_.size()); " << n_lanes_ = std::min(4, markings_.size()); << std::endl;
  //     delete lanes_[i];
  //     lanes_.erase(lanes_.begin() + i);
  //     i --;
  //     n_lanes = std::min(4, markings_.size()); --;
  //     //std::cout << "number of lanes " << lanes_.size()<< std::endl;
  //     //std::cout << "end  found one wrong lane " << i << " children " << lanes_[i]->Children().size() << " n_lanes = std::min(4, markings_.size()); " << n_lanes = std::min(4, markings_.size()); << std::endl;
  //   }
  // }
  for (auto lane: lanes_) {
    utility::Print(lane->ToString());
    for (int i = 0; i < 2; i++) {
      utility::Print(lane->Child(i)->ToString());
    }
  }

  std::cout << std::endl;
  Label* lane_label = grammar_->GetLabelByString("lane");
  int good_lane = 0;
  for (std::vector<LaneNode*>::iterator it = lanes_.begin(); it != lanes_.end(); it++) {
    LaneNode* l = *it;
    std::cout << l->ToString() << " ->";
    for (auto boundary: l->Children()) {
      std::cout <<  " " << boundary->ToString();
    }
    std::cout <<  std::endl;
    bool is_lane = lane_label->IsLabel(l);
    std::cout <<  " likelihood " << lane_label->ComputeEnergy(l->Value()) << " " << is_lane << std::endl;
    if (is_lane) {
      good_lane++;
    }
    else {
      lanes_.erase(it);
      it--;
      utility::PrintWarning("bad lane value");
      for (auto v: l->Value()) {
        std::cout << v << " ";
      }
      std::cout << std::endl;
    }
  }
  std::cout << " good_lane " << good_lane << std::endl;
  std::cout << std::endl;
  // return;

  /*********************************************************************************************
    roads layer. combination generates road 
  *********************************************************************************************/
  std::cout << "\033[1;33mFifth layer:\033[0m" << " roads" << std::endl;
  // 5 lanes maximum
  int max_lanes = std::min(5, (int)markings_.size()-1);
  int n_deleted_road = 0;
  for (int i = 0; i < max_lanes; i++) {
    std::vector<std::string> children(i + 1, "lane");
    road_rules.emplace("road", children);
    std::cout << "new rule road -> " << i + 1  << " lane" << std::endl;
  }
  for (auto rule: road_rules) {
    // we have one rule with fix cardinality
    int n_children = rule.second.size();
    // std::cout << "n_children " << n_children << std::endl;
    std::vector<std::vector<int> > combi = utility::Combination(lanes_.size(), n_children);
    int n_road = combi.size();
    for (int i = 0; i < n_road; i ++) {
      // in the loop, each one is a road
      RoadNode* new_road = new RoadNode(&graph_, node_index);
    	Label* label = grammar_->GetLabelByString("road");
      Rule* rule = grammar_->GetRulesDerivedFromLabel(label).front();
      new_road->SetAssociatedLabel(label);
      //new_road->SetAssociatedRule(rule);
      bool should_delete = false;
      std::vector<int> shared_boundary_count(boundaries_.size(), 0);
      for (auto index: combi[i]) {
        new_road->AddLane(lanes_[index-1]); 
      }

      Label* lane_label = grammar_->GetLabelByString("lane");
      Rule* lane_rule = grammar_->GetRulesDerivedFromLabel(label).front();
      LabelEdge* pLabel_edge = grammar_->GetLabelEdgeByInstance(lane_rule, lane_label, lane_label);

      for (int j = 0; j < new_road->NChildren(); j++) {
        if (!new_road->HasSiblingInChildren(new_road->Child(j))) {
          //std::cout << "old code is deleting" << std::endl;
          should_delete = true;
          break; 
        }
      }
      // add siblings
      if (should_delete) {
        graph_.EraseNode(new_road);
        n_deleted_road ++;
        continue;
      }
      roads_.push_back(new_road);
      node_index++;
    }
  }
  std::cout << std::endl;
  for (auto r: roads_) {
    std::cout << r->ToString() << " -> ";
    for (auto lane: r->Children()) {
      std::cout <<  " " << lane->ToString();
    }
    std::cout <<  std::endl;
  }
  std::cout << "size of roads " << roads_.size() << std::endl;
  std::cout << std::endl;
  // delete by completeness
  /*
  when a road has n lanes, it will have n + 1 lane boundaries
  */
  int good_road = 0;
  for (std::vector<RoadNode*>::iterator it = roads_.begin(); it != roads_.end(); it++) {
    bool should_delete = false;
    std::vector<int> lane_boundary_count(lane_boundaries_.size(), 0);
    for (auto lane: (*it)->Children()) {
      for (auto lane_child: lane->Children()) {
        LaneBoundaryNode* lane_boundary = dynamic_cast<LaneBoundaryNode*>(lane_child);
        std::vector<LaneBoundaryNode*>::iterator it = std::find(lane_boundaries_.begin(), lane_boundaries_.end(), lane_boundary);
        int index = std::distance(lane_boundaries_.begin(), it);
        lane_boundary_count[index] ++;
      }
    }
    int n_lane_boundary = 0;
    for (auto count: lane_boundary_count) {
      if (count > 0) {
        n_lane_boundary ++;
        if (count > 2) {
          should_delete = true;
          break;
        }
      }
    }
    if (n_lane_boundary != ((*it)->NChildren() + 1)) {
      should_delete = true;
    }
    if (should_delete ) {
      roads_.erase(it);
      it--;
    }
    else {
      good_road ++;
    }
  }

  std::cout << " good_road1 " << good_road << std::endl;
  good_road = 0;
  std::cout << std::endl;
  //delete by Label
  Label* road_label = grammar_->GetLabelByString("road");
  for (std::vector<RoadNode*>::iterator it = roads_.begin(); it != roads_.end(); it++) {
    RoadNode* r = *it;
    std::cout << r->ToString() << " ->";
    for (auto lane: r->Children()) {
      std::cout <<  " " << lane->ToString();
    }
    bool is_road = road_label->IsLabel(r);
    std::cout <<  " likelihood " << road_label->ComputeEnergy(r->Value()) << " " << is_road << std::endl;
    if (is_road) {
      good_road++;
    }
    else {
      roads_.erase(it);
      it--;
    }
  }
  std::cout << " good_road2 " << good_road << std::endl;
  std::cout <<  std::endl;
  std::cout <<  std::endl;

  /*********************************************************************************************
    scene layer. combination generates scene 
  *********************************************************************************************/
  std::cout << "\033[1;33mSixth layer\033[0m" << " scene" << std::endl;
  // two roads maximum
  int max_road = 2;
  int n_deleted_scene = 0;
  for (int i = 0; i < max_road; i++) {
    std::vector<std::string> children(i + 1, "road");
    scene_rules.emplace("scene", children);
    std::cout << "new rule scene -> " << i + 1  << " road" << std::endl;
  }
  for (auto rule: scene_rules) {
    // we have one rule with fix cardinality
    int n_children = rule.second.size();
    std::vector<std::vector<int> > combi = utility::Combination(roads_.size(), n_children);
    int n_scene = combi.size();
    for (int i = 0; i < n_scene; i ++) {
      // in the loop, each one is a road
      SceneNode* new_scene = new SceneNode(&graph_, node_index);
    	Label* label = grammar_->GetLabelByString("scene");
      Rule* rule = grammar_->GetRulesDerivedFromLabel(label).front();
      new_scene->SetAssociatedLabel(label);
      //new_road->SetAssociatedRule(rule);
      bool should_delete = false;
      for (auto index: combi[i]) {
        new_scene->AddRoad(roads_[index-1]); 
      }
      Label* road_label = grammar_->GetLabelByString("road");
      Rule* road_rule = grammar_->GetRulesDerivedFromLabel(label).front();
      LabelEdge* pLabel_edge = grammar_->GetLabelEdgeByInstance(road_rule, road_label, road_label);
      if (new_scene->NChildren() == 2) {
        // need to check edge
        if (!pLabel_edge->IsEdge(new_scene->Child(0), new_scene->Child(1))) {
          std::cout << "new code is deleting" << new_scene->ToString() << std::endl;
          should_delete = true;
          break; 
        }
      }
      if (should_delete) {
        graph_.EraseNode(new_scene);
        n_deleted_scene ++;
        continue;
      }
      scenes_.push_back(new_scene);
      node_index++;
    } 
  }
  std::cout << "n scene " << scenes_.size() << std::endl;
  std::cout << std::endl;

  // Print
  for (auto r: roads_) {
    // road
    std::cout << r->ToString() << " ";
    // lanes
    std::cout << "|| ";
    for (auto lane: r->Children()) {
      std::cout << lane->ToString() << " | ";
    }
    std::cout << "|| ";
    // boundaries
    for (auto lane: r->Children()) {
      for (auto b: lane->Children()) {
        std::cout << b->ToString() << " ";
      }
      std::cout << " | " ;
    }
    std::cout << "|| ";
    // markings_
    for (auto lane: r->Children()) {
      for (auto b: lane->Children()) {
        for (auto m: b->Children()) {
          std::cout <<  " " << m->ToString();
        }
        std::cout << " |" ;
      }
      std::cout << "|";
    }
    std::cout << "||";
    std::cout << std::endl << std::endl;
  }
  std::cout << "finished Combination Parse" << std::endl;

}
/*
  Breadth-first search. https://www.geeksforgeeks.org/breadth-first-search-or-bfs-for-a-graph/
*/
std::vector<BoundaryNode*> Parser::BFSAddBoundary() {
  if (markings_.size() == 0) {
    std::cout << "RecursiveAddBoundary has no marking" << std::endl;
    return std::vector<BoundaryNode*>{NULL};
  }
  std::vector<BoundaryNode*> boundaries;
  std::vector<std::vector<MarkingNode*> > marking_boundaries;
  std::vector<BoundaryNode*> queue;
  std::vector<bool> visited_boolean(markings_.size(), false);
  std::vector<std::vector<MarkingNode*> > non_sibling;
  non_sibling.resize(markings_.size());
  for (std::vector<MarkingNode*>::iterator i = markings_.begin(); i != markings_.end(); i ++) {
    BoundaryNode* one_boundary = NULL;
    std::cout << "new marking " << (*i)->ToString() << std::endl;
	  if (BFSAddOneBoundary(visited_boolean, i, one_boundary)) {
      if (one_boundary) {
        boundaries.push_back(one_boundary);
      }
    }
  }
  std::cout << "there are " << boundaries.size() << " marking boundaries" << std::endl;
  return boundaries;
}

bool Parser::BFSAddOneBoundary(std::vector<bool> &visited, std::vector<MarkingNode*>::iterator i, BoundaryNode* &out) {
  int index = std::distance(markings_.begin(), i);
	if (visited[index]) {
    std::cout << "return at vsitied" << std::endl<< std::endl;
    return false;
  }
  std::vector<MarkingNode*> markings;
	// Create a queue for BFS 
	//std::list<int> queue; 
  std::list<MarkingNode*> queue;

	// Mark the current node as visited and enqueue it 
	visited[index] = true; 
	queue.push_back(*i); 
  std::list<MarkingNode*>::iterator local;
	// 'i' will be used to get all adjacent 
	// vertices of a vertex 
	std::list<MarkingNode>::iterator it; 

	while(!queue.empty()) 
	{ 
		// Dequeue a vertex from queue and print it 
		local = queue.begin(); 
    markings.push_back(*local);
		// Get all adjacent vertices of the dequeued 
		// vertex s. If a adjacent has not been visited, 
		// then mark it visited and enqueue it 
    for (std::vector<MarkingNode*>::iterator itit = markings_.begin(); itit != markings_.end(); itit++) {
      if ((*local)->IsSibling(*itit)) {
			  if (!visited[std::distance(markings_.begin(), itit)]) {
          visited[std::distance(markings_.begin(), itit)] = true; 
				  queue.push_back(*itit); 
        }
      }
      else {
        //std::cout << (*local)->ToString() <<" " << (*itit)->ToString() << " are not sibling " << std::endl;
      }
    }
		queue.pop_front(); 
	}
  out = new BoundaryNode(&graph_, graph_.NNodes());
  Label* label = grammar_->GetLabelByString("marking_boundary");
  out->SetAssociatedLabel(label);
  Label* pre_label = grammar_->GetLabelByString("lane_boundary");
  Rule* rule = grammar_->GetRulesDerivedFromLabel(pre_label).front();
  out->SetAssociatedRule(rule);
  // build boundary
  for (auto marking: markings) {
    out->AddMarking(marking);
  }
  return true;
}

void Parser::GenerateEdgeInMarkings() {
  /*********************************************************************************************
    markings layer. add edge/ sibling between markings
  *********************************************************************************************/
  std::cout << "\033[1;33mFirst layer: Markings\033[0m" << std::endl;
  for (std::vector<MarkingNode*>::iterator it = markings_.begin(); it != markings_.end(); it++) {
    MarkingNode* pnode = *it;
    for (std::vector<MarkingNode*>::iterator itit = it + 1; itit != markings_.end(); itit++) {
      GraphNode* sibling = dynamic_cast<GraphNode*>(*itit);
      pnode->AddSibling(sibling);
    }
  }
  for (std::vector<MarkingNode*>::iterator it = markings_.begin(); it != markings_.end(); it++) {
    MarkingNode* pnode = *it;
    pnode->PrintSibling();
  }
  std::cout << std::endl;
}

void Parser::GenerateBoundaries() {
  /*********************************************************************************************
    boundaries layer
  *********************************************************************************************/
  std::cout << "\033[1;33mSecond layer: Boundaries\033[0m" << std::endl;
  std::vector<BoundaryNode*> potential_boundaries = BFSAddBoundary();
  std::vector<BoundaryNode*> final_boundaries;
  std::cout << "number of potential_boundaries " << potential_boundaries.size() << std::endl;
  if (is_visual_) {
    Visualization potential_boundaries_visual(&graph_);
    cv::Mat potential_boundaries_image = potential_boundaries_visual.DrawBoundary(potential_boundaries);
    cv::Mat potential_markings_image = potential_boundaries_visual.DrawMarkings();
	  imwrite("0potential_boundaries_image.png", potential_boundaries_image);
	  imwrite("0potential_markings_image.png", potential_markings_image);
  }
  Label* boundary_label = grammar_->GetLabelByString("marking_boundary");
  for (std::vector<BoundaryNode*>::iterator potential_boundary_it = potential_boundaries.begin(); potential_boundary_it != potential_boundaries.end();
     potential_boundary_it ++) {
    if (boundary_label->IsMinimalLabel(*potential_boundary_it)) {
      std::cout << "potential_boundary is label, push back" << std::endl;;
      final_boundaries.push_back(*potential_boundary_it);
    }
    else {
      // potential boundary is not a valid boundary, probably has some miss classified marking in the node.
      if ((*potential_boundary_it)->NChildren() == 1) {
        graph_.EraseBoundaryNode(*potential_boundary_it);
        graph_.EraseNode(*potential_boundary_it);
        continue;
      }
      /*
        Three ways to remove false positive
      */
      int n_semantic_deleted_boundary = 0;
      int n_label_deleted_boundary = 0;
      int n_subset_deleted_boundary = 0;

      //std::vector<MarkingNode*> markings = graph_.GetMarkings();
      std::vector<MarkingNode*> markings;
      // temporary vector contains BoundaryNode checked by semantic and label 
      std::vector<BoundaryNode*> tem_boundaries;
      for (auto c: (*potential_boundary_it)->Children()) {
        markings.push_back(dynamic_cast<MarkingNode*>(c));
      }
      std::multimap<std::string, std::vector<std::string> > boundary_rules;
      for (int i = 0; i < markings.size(); i++) {
        std::vector<std::string> children(i + 1, "marking");
        boundary_rules.emplace("boundary", children);
        std::cout << "new rule boundary -> " << i + 1  << " marking" << std::endl;
      }
      bool interesting = true;
      int n1 = 0;
      for (auto rule: boundary_rules) {
        n1 = tem_boundaries.size();
        std::cout << n1 << std::endl;
        // we have one rule with fix cardinality, so find the number of markings and insert as children
        int n_children = rule.second.size();
        std::vector<std::vector<int> > combi = utility::Combination(markings.size(), n_children);
        int n_boundary = combi.size();
        for (int i = 0; i < n_boundary; i ++) {
          // in the loop, each one is a boundary
          BoundaryNode* new_boundary = new BoundaryNode(&graph_, graph_.NNodes());
          new_boundary->SetAssociatedLabel(boundary_label);
        	Label* pre_label = grammar_->GetLabelByString("lane_boundary");
          Rule* rule = grammar_->GetRulesDerivedFromLabel(pre_label).front();
          new_boundary->SetAssociatedRule(rule);
          // add all the children from combi
          for (auto index: combi[i]) {
            new_boundary->AddMarking(markings[index-1]); 
          }
          if (!boundary_label->IsLabel(new_boundary)) {
            // label check
            graph_.EraseBoundaryNode(new_boundary);
            graph_.EraseNode(new_boundary);
            n_label_deleted_boundary ++;
            continue;
          }
          // if a child has no siblings among the children_, delete this node
          bool should_delete = false;
          for (int j = 0; j < new_boundary->NChildren(); j++) {
            if (!new_boundary->HasSiblingInChildren(new_boundary->Child(j))) {
              //std::cout << " don't HasSiblingInChildren breaking..." << std::endl;
              should_delete = true;
              break;
            }
          }
          if (should_delete) {
            graph_.EraseBoundaryNode(new_boundary);
            graph_.EraseNode(new_boundary);
            n_semantic_deleted_boundary ++;
          }
          else {
            tem_boundaries.push_back(new_boundary);
            std::cout << "new boundary " << new_boundary->ToString() << std::endl;
          }
        }
        // early break when no more boundary is added into tem_boundaries, which means no more valid boundaries will be generated
        if (n1 == tem_boundaries.size() && n1 != 0) {
          break;
        }
      }
      std::set<BoundaryNode*> delete_queue_boundaries;
      std::cout << "number of boundaries " << tem_boundaries.size() << " n_semantic_deleted_boundary " << n_semantic_deleted_boundary
        << " n_label_deleted_boundary " << n_label_deleted_boundary << std::endl;

      for (std::vector<BoundaryNode*>::iterator out_it = tem_boundaries.begin(); out_it != tem_boundaries.end(); out_it ++) {
        for (std::vector<BoundaryNode*>::iterator in_it = tem_boundaries.begin(); in_it != tem_boundaries.end(); in_it ++) {
          if ((*out_it)->IsSubset(*in_it)) {
            delete_queue_boundaries.insert(*in_it);
          }
        }
      }
      for (std::vector<BoundaryNode*>::iterator tem_it = tem_boundaries.begin(); tem_it != tem_boundaries.end(); tem_it ++) {
        std::set<BoundaryNode*>::iterator delete_it = std::find(delete_queue_boundaries.begin(), delete_queue_boundaries.end(), *tem_it);
        if (delete_it == delete_queue_boundaries.end()) {
          final_boundaries.push_back(*tem_it);
        }
        else {
          graph_.EraseBoundaryNode(*tem_it);
          graph_.EraseNode(*tem_it);
          n_subset_deleted_boundary ++;
        }
      }
      graph_.EraseBoundaryNode(*potential_boundary_it);
      graph_.EraseNode(*potential_boundary_it);
      std::cout << "number of tem_boundaries " << tem_boundaries.size() << " n_subset_deleted_boundary " << delete_queue_boundaries.size() << std::endl;
    }
  }
  utility::PrintGreen("n final_boundaries",final_boundaries.size());
  utility::PrintGreen("n boundaries in graph_",graph_.GetBoundaries().size());

  for (std::vector<BoundaryNode*>::iterator it = final_boundaries.begin(); it != final_boundaries.end(); it ++) {
    std::cout << (*it)->ToString() << std::endl;
    std::vector<GraphNode*> children = (*it)->Children();
    for (auto c: children) {
      std::cout << c->ToString() << " ";
    }
    std::cout << std::endl;    
  }
  if (is_visual_) {
    Visualization boundary_visual(&graph_);
    cv::Mat boundary_image = boundary_visual.DrawBoundary();
    cv::Mat final_boundaries_image = boundary_visual.DrawBoundary(final_boundaries);
	  imwrite("0parser_boundary.png", boundary_image);
	  imwrite("0final_boundaries.png", final_boundaries_image);
  }
  /**********************************************************************************************
    assign boundaries to candidate graphs, so boundary splitting can deal with false negative in 
    boundaries_, and generates correct lane boundaries
  **********************************************************************************************/
  const int n_marking_boundaries = final_boundaries.size();
  std::multimap<std::string, std::vector<std::string> > graph_boundary_rules;
  int n_max_markingboundaries = std::min(5, n_marking_boundaries);
  for (int i = 0; i < n_max_markingboundaries + 1; i++) {
    std::vector<std::string> children(i, "marking boundary");
    graph_boundary_rules.emplace("scene", children);
    std::cout << "new rule scene -> " << i << " marking boundary" << std::endl;
  }
  int n_graph = 0;
  int graph_index = 0;
  for (auto rule: graph_boundary_rules) {
    /* 
      each rule with fix cardinality. Fixed size of boundaries in a graph. Add those boundaries to a graph
    */
    int n_children = rule.second.size();
    std::vector<std::vector<int> > combi = utility::Combination(n_marking_boundaries, n_children);
    for (int i = 0; i < combi.size(); i ++, graph_index ++) {
      // in the loop, each one is a new graph, with different boundaries
      Graph* new_graph = new Graph(graph_index);
      new_graph->SetGrammar(grammar_);
      // bug
      *new_graph = base_graph_;
      for (auto index: combi[i]) {
        Polyline polyline = final_boundaries.at(index -1)->GetPolyline();
        BoundaryNode* new_boundary = new BoundaryNode(new_graph, new_graph->NNodes(), polyline);
        new_boundary->SetAssociatedLabel(boundary_label);
    	  Label* pre_label = grammar_->GetLabelByString("lane");
        Rule* rule = grammar_->GetRulesDerivedFromLabel(pre_label).front();
        new_boundary->SetAssociatedRule(rule);
      }
      candidate_graphs_.push_back(new_graph);
    }
    n_graph += combi.size();
  }
  std::vector<bool> initial_value(n_graph, true);
  complete_graph_boolean_ = initial_value;
  int n_uncomplete = 0;
  for (int i = 0; i < candidate_graphs_.size(); i++) {
    Graph* graph = candidate_graphs_[i];
    if (graph->IsBoundariesIntersect()) {
      complete_graph_boolean_[i] = false;
      n_uncomplete ++;
    }
  }
  utility::PrintGreen("n uncomplete graphs", n_uncomplete);
  utility::PrintGreen("n graphs", n_graph);
  std::cout << std::endl;
}

void Parser::GenerateLaneBoundaries() {
  /*********************************************************************************************
    lane boundaries layer. counter generates lane boundaries
  *********************************************************************************************/
  std::cout << "\033[1;33mThird layer: Lane Boundaryies\033[0m" << std::endl;
  std::cout << "Third layer has " << candidate_graphs_.size() << " graphs" << std::endl;
  for (int i = 0; i < candidate_graphs_.size(); i++) {
    Graph* graph = candidate_graphs_[i];
    utility::PrintGreen(graph->ToString());
    if (!complete_graph_boolean_[i]) {
      utility::Print("uncomplete graph");      
      continue;
    }
    graph->GenerateLaneBoundary();
    if (graph->NRoads() > 2) {
      // only deal with one splitting or merge
      complete_graph_boolean_[i] = false;
      utility::PrintGreen("too many roads", graph->NRoads());      
      continue;
    }
    std::vector<LaneBoundaryNode*> lane_boundaries = graph->GetLaneBoundaries();
    utility::Print(graph->ToString() + " has n lane boundaries", lane_boundaries.size());
    for (auto lane_boundary1: lane_boundaries) {
      for (auto lane_boundary2: lane_boundaries) {
        if (lane_boundary1->AddSibling(lane_boundary2)) {
          std::cout << lane_boundary1->ToString() << " and " << lane_boundary2->ToString() << " are siblings" << std::endl;    
        }
      }
    }    
    utility::PrintGreen(graph->ToString() + " has n lane boundaries", lane_boundaries.size());
    utility::PrintGreen(graph->ToString() + " has n lane boundaries", graph->GetLaneBoundaries().size());
  }
  std::cout << std::endl;
}

void Parser::GenerateLanes() {
  /*********************************************************************************************
    lane layer. combination generates lane 
  *********************************************************************************************/
  std::cout << "\033[1;33mFourth layer: Lanes\033[0m" << std::endl;
  std::vector<std::string> children(2, "lane_boundary");
  std::cout << "new rule lane -> lane_boundary lane_boundary" << std::endl;
  for (int i = 0; i < candidate_graphs_.size(); i++) {
    Graph* graph = candidate_graphs_[i];
    utility::PrintGreen(graph->ToString());
    if (!complete_graph_boolean_[i]) {
      utility::Print("uncomplete graph");      
      continue;
    }
    int n_deleted_lane = 0;
    // we have one rule with fix cardinality 2
    std::vector<LaneBoundaryNode*> lane_boundaries = graph->GetLaneBoundaries();
    std::vector<std::vector<int> > combi = utility::Combination(lane_boundaries.size(), 2);
    // combi.size() is number of lanes
    for (int i = 0; i < combi.size(); i ++) {
      // each loop is a boundary
      int child_index_1 = combi[i][0] - 1;
      int child_index_2 = combi[i][1] - 1;
      LaneBoundaryNode* child1 = lane_boundaries[child_index_1];
      LaneBoundaryNode* child2 = lane_boundaries[child_index_2];
      if (child1->IsSibling(child2)) {
        // if those two are siblings
        LaneNode* new_lane = new LaneNode(graph, graph->NNodes());
        Label* label = grammar_->GetLabelByString("lane");
        new_lane->SetAssociatedLabel(label);
        Label* pre_label = grammar_->GetLabelByString("road");
        Rule* rule = grammar_->GetRulesDerivedFromLabel(pre_label).front();
        new_lane->SetAssociatedRule(rule);      
        new_lane->AddBoundary(child1);
        new_lane->AddBoundary(child2);
        std::vector<LaneNode*> partial_lanes = graph->GetLanes();
        for (auto l: partial_lanes) {
          new_lane->AddSibling(l);
        }
      }
    }
    std::vector<LaneNode*> lanes = graph->GetLanes();    
    Label* lane_label = grammar_->GetLabelByString("lane");
    int good_lane = 0;
    for (std::vector<LaneNode*>::iterator lane_it = lanes.begin(); lane_it != lanes.end(); lane_it ++) {
      std::cout << (*lane_it)->ToString() << " ->";
      for (auto boundary: (*lane_it)->Children()) {
        std::cout <<  " " << boundary->ToString();
      } 
      bool is_lane = lane_label->IsLabel((*lane_it));
      std::cout <<  " likelihood " << lane_label->ComputeEnergy((*lane_it)->Value()) << " " << is_lane << std::endl;
      if (is_lane) {
        good_lane++;
      }
      else {
        utility::Print(" lane value", (*lane_it)->ToString());
        for (auto v: (*lane_it)->Value()) {
          std::cout << v << " ";
        }
        std::cout << std::endl;
        graph->EraseLaneNode(*lane_it);
      }
    }
    utility::PrintGreen(graph->ToString() + " has n lanes", graph->GetLanes().size());
    /*
      one road: n lanes and (n+1) lane boundaries
      two roads: n lanes and (n+2) lane boundaries
    */
    if (lanes.size() + graph->NRoads() < lane_boundaries.size()) {
      utility::PrintError("uncomplete, should break");
      complete_graph_boolean_[i] = false;
    }
    std::cout << std::endl;
  }
}

void Parser::GenerateRoads() {
  /*********************************************************************************************
    roads layer. combination generates road 
  *********************************************************************************************/
  std::cout << "\033[1;33mFifth layer: Roads\033[0m" << std::endl;

  for (int i = 0; i < candidate_graphs_.size(); i++) {
    Graph* graph = candidate_graphs_[i];
    utility::PrintGreen(graph->ToString());
    if (!complete_graph_boolean_[i]) {
      utility::Print("uncomplete graph");      
      continue;
    }
    std::vector<LaneNode*> lanes = graph->GetLanes();
    if (lanes.size() == 0) {
      // TODO: partial parsing
      utility::Print("graph has no lane");      
      continue;
    }
    std::multimap<std::string, std::vector<std::string> > road_rules;
    int max_lanes = std::min(5, (graph->NBoundaries() - 1));
    int n_deleted_road = 0;
    for (int i = 0; i < max_lanes; i++) {
      std::vector<std::string> children(i + 1, "lane");
      road_rules.emplace("road", children);
      std::cout << "new rule road -> " << i + 1  << " lane" << std::endl;
    }
    for (auto rule: road_rules) {
      // we have one rule with fix cardinality
      int n_children = rule.second.size();
      // std::cout << "n_children " << n_children << std::endl;
      std::vector<std::vector<int> > combi = utility::Combination(lanes.size(), n_children);
      int n_road = combi.size();
      for (int i = 0; i < n_road; i ++) {
        // in the loop, each one is a road
        RoadNode* new_road = new RoadNode(graph, graph->NNodes());
      	Label* label = grammar_->GetLabelByString("road");
        Rule* rule = grammar_->GetRulesDerivedFromLabel(label).front();
        new_road->SetAssociatedLabel(label);
        //new_road->SetAssociatedRule(rule);
        for (auto index: combi[i]) {
          new_road->AddLane(lanes[index-1]); 
        }

        Label* lane_label = grammar_->GetLabelByString("lane");
        Rule* lane_rule = grammar_->GetRulesDerivedFromLabel(label).front();
        LabelEdge* pLabel_edge = grammar_->GetLabelEdgeByInstance(lane_rule, lane_label, lane_label);

        for (int j = 0; j < new_road->NChildren(); j++) {
          if (!new_road->HasSiblingInChildren(new_road->Child(j))) {
            graph->EraseRoadNode(new_road);
            n_deleted_road ++;
            break; 
          }
        }
      }
    }
    std::vector<RoadNode*> roads = graph->GetRoads();
    std::vector<LaneBoundaryNode*> lane_boundaries = graph->GetLaneBoundaries();
    std::cout << "size of roads " << roads.size() << std::endl;
    std::cout << "size of lane_boundaries " << lane_boundaries.size() << std::endl;
    /*
      discard by completeness
      when a road has n lanes, it will have n + 1 lane boundaries
    */
    int good_road = 0;
    for (std::vector<RoadNode*>::iterator it = roads.begin(); it != roads.end(); it++) {
      bool should_delete = false;
      std::vector<int> lane_boundary_count(lane_boundaries.size(), 0);
      for (auto lane: (*it)->Children()) {
        for (auto lane_child: lane->Children()) {
          LaneBoundaryNode* lane_boundary = dynamic_cast<LaneBoundaryNode*>(lane_child);
          std::vector<LaneBoundaryNode*>::iterator it = std::find(lane_boundaries.begin(), lane_boundaries.end(), lane_boundary);
          int index = std::distance(lane_boundaries.begin(), it);
          lane_boundary_count[index] ++;
        }
      }
      int n_lane_boundary = 0;
      for (auto count: lane_boundary_count) {
        if (count > 0) {
          n_lane_boundary ++;
          if (count > 2) {
            should_delete = true;
            break;
          }
        }
      }
      for (auto l1: (*it)->Children()) {
        LaneNode* lane1 = dynamic_cast<LaneNode*>(l1);
        Polygon polygon = lane1->GetPolygon();
        for (auto l2: (*it)->Children()) {
          if (l1 == l2) continue;
          LaneNode* lane2 = dynamic_cast<LaneNode*>(l2);
          Point center_point = lane2->GetCenterPoint();
          if (polygon.GetWidthAtPoint(center_point) != INT_MAX) {
            should_delete = true;
            break;
          }
        }
        if (should_delete) {
          break;
        }
      }
      if (should_delete) {
        graph->EraseRoadNode(*it);
      }
      else {
        good_road ++;
      }
    }
    std::cout << " good_road1 " << good_road << std::endl;
    good_road = 0;
    //delete by Label
    roads = graph->GetRoads();
    Label* road_label = grammar_->GetLabelByString("road");
    for (std::vector<RoadNode*>::iterator it = roads.begin(); it != roads.end(); it++) {
      RoadNode* r = *it;
      std::cout << r->ToString() << " ->";
      for (auto lane: r->Children()) {
        std::cout <<  " " << lane->ToString();
      }
      bool is_road = road_label->IsLabel(r);
      std::cout <<  " likelihood " << road_label->ComputeEnergy(r->Value()) << " " << is_road << std::endl;
      if (is_road) {
        good_road++;
      }
      else {
        graph->EraseRoadNode(*it);
        roads.erase(it);
        it--;
      }
    }
    std::cout << " good_road2 " << good_road << std::endl;
    std::cout <<  std::endl;
  }
} 

void Parser::GenerateScenes() {
  /*********************************************************************************************
    scene layer. combination generates scene 
  *********************************************************************************************/
  std::cout << "\033[1;33mSixth layer: Scene\033[0m" << std::endl;

  for (int graph_index = 0; graph_index < candidate_graphs_.size(); graph_index ++) {
    Graph* graph = candidate_graphs_[graph_index];
    utility::PrintGreen(graph->ToString());
    if (!complete_graph_boolean_[graph_index]) {
      utility::Print("uncomplete graph", graph_index);      
      continue;
    }
    int n_deleted_scene = 0;
    std::multimap<std::string, std::vector<std::string> > scene_rule;
    std::vector<RoadNode*> roads = graph->GetRoads();
    // two roads maximum
    int n_roads = roads.size();
    int max_road = 2;
    // for (int i = 0; i < std::min(max_road, n_roads) ; i++) {
    //   std::vector<std::string> children(i + 1, "road");
    //   scene_rules.emplace("scene", children);
    //   std::cout << "new rule scene -> " << i + 1  << " road" << std::endl;
    // }
    std::vector<std::string> children(graph->NRoads(), "road");
    scene_rule.emplace("scene", children);
    std::cout << "new rule scene -> " << graph->NRoads()  << " road" << std::endl;
    if (roads.size() == 0) {
      // TODO: partial parsing
      utility::Print("graph has no roads");
      continue;
    }
    for (auto rule: scene_rule) {
      // we have one rule with fix cardinality
      int n_children = rule.second.size();
      std::vector<std::vector<int> > combi = utility::Combination(roads.size(), n_children);
      int n_scene = combi.size();
      std::cout << "n_scene " << n_scene << std::endl;
      for (int i = 0; i < n_scene; i ++) {
        // in the loop, each one is a scene node
        SceneNode* new_scene = new SceneNode(graph, graph->NNodes());
      	Label* scene_label = grammar_->GetLabelByString("scene");
        Rule* rule = grammar_->GetRulesDerivedFromLabel(scene_label).front();
        new_scene->SetAssociatedLabel(scene_label);
        //new_road->SetAssociatedRule(rule);
        bool should_delete = false;
        for (auto index: combi[i]) {
          new_scene->AddRoad(roads[index-1]); 
        }
        std::cout << new_scene->ToString() << " ->";
        for (auto road: new_scene->Children()) {
          std::cout <<  " " << road->ToString();
        }
        std::cout << std::endl;
        for (auto c: new_scene->Value()) {
          std::cout <<  " new_scene->Value() " << c << std::endl;
        }
        std::cout <<  " likelihood " << scene_label->ComputeEnergy(new_scene->Value()) << std::endl;
        if (scene_label->IsLabel(new_scene)) {
          utility::Print("new scene", new_scene->ToString());
        }
        else {
          utility::Print("label is deleting scene", new_scene->ToString());
          graph->EraseSceneNode(new_scene);
          n_deleted_scene ++;
        }
      }
    }
    std::cout << "n scene " << graph->GetScenes().size() << std::endl;
    std::cout << "n deleted scene " << n_deleted_scene << std::endl;

    std::cout << "semantic check scene" << std::endl;
    std::vector<SceneNode*> scenes = graph->GetScenes();
    for (auto scene: scenes) {
      if (!scene->CheckSemantic()) {
        graph->EraseSceneNode(scene);
      }
    }
    std::cout << "scene end" << std::endl;
    std::cout << std::endl;
  }
}

void Parser::BottomUpParse() {
  GenerateEdgeInMarkings();
  GenerateBoundaries();
  GenerateLaneBoundaries();
  GenerateLanes();
  GenerateRoads();
  GenerateScenes();
}

void Parser::Log(const std::chrono::milliseconds duration, const int graph_index, const int clutter_type) const {
  std::cout << "duration: " << duration.count() << std::endl; 
  std::cout << "graph_index: " << graph_index << std::endl; 
  // std::ofstream file1; 
  // file1.open("time.txt");
  // file1 << graph_.GetMarkings().size();
  // file1 << ' ';
  // file1 << duration.count();
  // file1 << '\n';
  // file1.close(); 
	std::ifstream input;
	input.exceptions(std::ifstream::badbit | std::ifstream::failbit);
  try {
    input.open ("time" + std::to_string(clutter_type) + ".json");
  }
  catch (std::ifstream::failure e) {
    std::cerr << "Exception opening file time.json" << std::endl << std::endl;
  }
  json json_graph;
	input >> json_graph;

  json_graph[std::to_string(graph_index)] = {
    {"n_marking", graph_.GetMarkings().size()},
    {"n_boundary",  graph_.GetBoundaries().size()},
    {"scene_size", graph_.SceneSize()},
    {"clutter_type", clutter_type},
		{"time", duration.count()}
  };
  std::ofstream output("time" + std::to_string(clutter_type) + ".json");
  output << std::setw(4) << json_graph << std::endl;
}

void Parser::SetVisual(const bool is_visual) {
  is_visual_ = is_visual;
}

} // ns