#include <assert.h>
#include <iomanip>
#include <iostream>
#include <fstream>
#include "Graph.h"
#include "Grammar.h"
#include "GraphNode.h"
#include "Label.h"
#include "Utility.h"
#include "json.hpp"

// to be removed
#include "Visualization.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using json = nlohmann::json;

namespace lane_model {

Graph::Graph(): scene_size_(0.0), n_roads_(1) {
	graph_boundaries_.reserve(2);
}

Graph::Graph(int graph_index): scene_size_(0.0), graph_index_(graph_index), n_roads_(1) {
	graph_boundaries_.reserve(2);
}

Graph::Graph(const Graph& other) {
	scene_size_ = other.scene_size_;
	offset_x_ = other.offset_x_;
	offset_y_ = other.offset_y_;
	n_roads_ = other.n_roads_;
	driving_direction_ = other.driving_direction_;
	graph_boundaries_ = other.graph_boundaries_;
	polygon_ = other.polygon_;
	graph_width_ = other.graph_width_;
	grammar_ = other.grammar_;
	graph_index_ = 0;

	for (auto m: other.markings_) {
		MarkingNode* marking_node = new MarkingNode(this, m->GraphIndex());
		*marking_node = *m;
	}
	for (auto b: other.boundaries_) {
		BoundaryNode* boundary_node = new BoundaryNode(this, b->GraphIndex());
		*boundary_node = *b;
	}
	for (auto lb: other.lane_boundaries_) {
		LaneBoundaryNode* lane_boundary_node = new LaneBoundaryNode(this, lb->GraphIndex());
		*lane_boundary_node = *lb;
	}
	for (auto l: other.lanes_) {
		LaneNode* lane_node = new LaneNode(this, l->GraphIndex());
		*lane_node = *l;
	}
	for (auto r: other.roads_) {
		RoadNode* road_node = new RoadNode(this, r->GraphIndex());
		*road_node = *r;
	}
	for (auto s: other.scenes_) {
		SceneNode* scene_node = new SceneNode(this, s->GraphIndex());
		*scene_node = *s;
	}
}

/*
	assignment doesn't change the graph index
*/
Graph& Graph::operator = (const Graph& other) {
	scene_size_ = other.scene_size_;
	offset_x_ = other.offset_x_;
	offset_y_ = other.offset_y_;
	n_roads_ = other.n_roads_;
	driving_direction_ = other.driving_direction_;
	graph_boundaries_ = other.graph_boundaries_;
	polygon_ = other.polygon_;
	graph_width_ = other.graph_width_;
	grammar_ = other.grammar_;

	for (auto m: other.markings_) {
		MarkingNode* marking_node = new MarkingNode(this, m->GraphIndex());
		*marking_node = *m;
	}
	for (auto b: other.boundaries_) {
		BoundaryNode* boundary_node = new BoundaryNode(this, b->GraphIndex());
		*boundary_node = *b;
	}
	for (auto lb: other.lane_boundaries_) {
		LaneBoundaryNode* lane_boundary_node = new LaneBoundaryNode(this, lb->GraphIndex());
		*lane_boundary_node = *lb;
	}
	for (auto l: other.lanes_) {
		LaneNode* lane_node = new LaneNode(this, l->GraphIndex());
		*lane_node = *l;
	}
	for (auto r: other.roads_) {
		RoadNode* road_node = new RoadNode(this, r->GraphIndex());
		*road_node = *r;
	}
	for (auto s: other.scenes_) {
		SceneNode* scene_node = new SceneNode(this, s->GraphIndex());
		*scene_node = *s;
	}
}

Graph::~Graph() {
	for (std::vector<GraphNode*>::iterator node = nodes_.begin(); node != nodes_.end(); node++) {
		delete *node;
		*node = NULL;
  }
  nodes_.clear();
}

void Graph::ImportGraph(const std::string& filename, int i) {
	graph_index_ = i;
  std::cout << "begin importing a graph " << filename << std::endl;
  
	std::ifstream input;
	input.exceptions(std::ifstream::badbit | std::ifstream::failbit);
  try {
    input.open (filename);
  }
  catch (std::ifstream::failure e) {
    std::cerr << "Exception opening file " << filename << std::endl << std::endl;
  }
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
  offset_x_ = *std::min_element(array_x.begin(), array_x.end()) - 10; 
  offset_y_ = *std::min_element(array_y.begin(), array_y.end()) - 10;
  double scene_width = *std::max_element(array_x.begin(), array_x.end()) - offset_x_;
  double scene_height = *std::max_element(array_y.begin(), array_y.end()) - offset_y_;
  scene_size_ = std::max(scene_width, scene_height);

  std::cout << " offset_x_ " << offset_x_ << " offset_y_ "<< offset_y_ << std::endl; 
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
    	    center.x = itit.value().get<double>() - offset_x_;
    	    // std::cout << "center.x  " << center.x << std::endl; 
    	  }
    	  else if(!itit.key().compare("center_y")) {
    	    center.y = itit.value().get<double>() - offset_y_;
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
    		  if (!label) {
    		    label = new Label(grammar_);
    		    label->SetBasicCategory(label_name);
    		  }
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

    	cv::RotatedRect cv_rect(center, size, angle);
			MarkingNode* marking_node = new MarkingNode(this, id, cv_rect);
			marking_node->SetAssociatedLabel(label);
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
    		        x = coordinate_it.value().get<double>() - offset_x_;
    		      }
    		      else if(!coordinate_it.key().compare("y")) {
    		        y = coordinate_it.value().get<double>() - offset_y_;
    		      }
    		    }
    		    points.emplace_back(x, y);
    		  }
    		}
				else if(!itit.key().compare("id")) {
    	    id = itit.value().get<int>();
    	  }
				else if(!itit.key().compare("label")) {
    		  //std::string label_name = itit.value().get<std::string>();
    		  std::string label_name = "marking_boundary";    		  
					label = grammar_->GetLabelByString(label_name);
    		  if (!label) {
    		    label = new Label(grammar_);
    		    label->SetBasicCategory(label_name);
    		  }
				}
			}
			Polyline polyline(points);
			BoundaryNode* boundary_node = new BoundaryNode(this, id, polyline);
			boundary_node->SetAssociatedLabel(label);

			AddGraphBoundary(polyline);
			// boundaries_.push_back(boundary_node);
			// boundary_node->SetPolyline(polyline);
			// std::cout << "boundaries push back at points boundary " << boundaries.size() << std::endl;		
    }
		else if (!it.key().compare(0, 16, "marking_boundary")) {
			// marking boundary node
			for (json::iterator itit = it->begin(); itit != it->end(); ++itit) {
        if(!itit.key().compare("id")) {
    	    id = itit.value().get<int>();
    	  }
			}
			BoundaryNode* boundary_node = new BoundaryNode(this, id);
			//boundaries_.push_back(boundary_node);
      for (json::iterator itit = it->begin(); itit != it->end(); ++itit) {
        if(!itit.key().compare("children")) {
					// marking boundary
     			std::stringstream stream(itit.value().get<std::string>());
      		int child_id;
      		while(stream >> child_id) {
						boundary_node->AddMarking(GetNodeWithIndex(child_id));
						//boundary_node->InsertChild(GetNodeWithIndex(child_id));
      		}
				}
				else if(!itit.key().compare("label")) {
    		  std::string label_name = itit.value().get<std::string>();
    		  Label* label = grammar_->GetLabelByString(label_name);
    		  if (!label) {
    		    label = new Label(grammar_);
    		    label->SetBasicCategory(label_name);
    		  }
					boundary_node->SetAssociatedLabel(label);
				}
			}
		}
		else if (!it.key().compare(0, 15, "n_lane_boundary")) {
      // lane boundary node
			std::vector<Point> points;
    	Label* label = NULL;
			int child_id;
			for (json::iterator itit = it->begin(); itit != it->end(); ++itit) {
    		if(!itit.key().compare("points")) {
    		  for (json::iterator point_it = itit->begin(); point_it != itit->end(); ++point_it) {
    		    double x, y;
    		    for (json::iterator coordinate_it = point_it->begin(); coordinate_it != point_it->end(); ++coordinate_it) {
    		      if(!coordinate_it.key().compare("x")) {
    		        //x = std::stod(coordinate_it.value().get<std::string>());
    		        x = std::stod(coordinate_it.value().get<std::string>()) - offset_x_;
    		      }
    		      else if(!coordinate_it.key().compare("y")) {
    		        //y = std::stod(coordinate_it.value().get<std::string>());
    		        y = std::stod(coordinate_it.value().get<std::string>()) - offset_y_;
    		      }
    		    }
    		    points.emplace_back(x, y);
    		  }
    		}
				else if(!itit.key().compare("children")) {
     			child_id = std::stoi(itit.value().get<std::string>());
				}
				else if(!itit.key().compare("id")) {
    	    id = itit.value().get<int>();
    	  }
				else if(!itit.key().compare("label")) {
    		  std::string label_name = itit.value().get<std::string>();
					label = grammar_->GetLabelByString(label_name);
    		  if (!label) {
    		    label = new Label(grammar_);
    		    label->SetBasicCategory(label_name);
    		  }
				}
			}
			Polyline polyline(points);
			LaneBoundaryNode* lane_boundary_node = new LaneBoundaryNode(this, id, polyline);
			lane_boundary_node->SetAssociatedLabel(label);

			lane_boundary_node->InsertChild(GetNodeWithIndex(child_id));
			//std::cout << "\033[1;31mLane boudnary add as child:  \033[0m" << child_id << std::endl;		
    }
		else if (!it.key().compare(0, 9, "road_lane")) {
			// lane node 
			for (json::iterator itit = it->begin(); itit != it->end(); ++itit) {
        if(!itit.key().compare("id")) {
    	    // std::cout << "id  " << itit.value().get<int>() << std::endl; 
    	    id = itit.value().get<int>();
    	  }
			}
      LaneNode* lane_node = new LaneNode(this, id);
      for (json::iterator itit = it->begin(); itit != it->end(); ++itit) {
        if(!itit.key().compare("children")) {
     			std::stringstream stream(itit.value().get<std::string>());
      		int child_id;
      		while(stream >> child_id) {
						lane_node->AddBoundary(GetNodeWithIndex(child_id));
      		}
				}
				else if(!itit.key().compare("label")) {
    		  std::string label_name = itit.value().get<std::string>();
    		  Label* label = grammar_->GetLabelByString(label_name);
    		  if (!label) {
    		    label = new Label(grammar_);
    		    label->SetBasicCategory(label_name);
    		  }
					lane_node->SetAssociatedLabel(label);
				}
			}
    }
		else if (!it.key().compare(0, 9, "road_road")) {
			for (json::iterator itit = it->begin(); itit != it->end(); ++itit) {
        if(!itit.key().compare("id")) {
    	    // std::cout << "id  " << itit.value().get<int>() << std::endl; 
    	    id = itit.value().get<int>();
    	  }		
			}
      RoadNode* road_node = new RoadNode(this, id);
      for (json::iterator itit = it->begin(); itit != it->end(); ++itit) {
        if(!itit.key().compare("children")) {
					// marking boundary
     			std::stringstream stream(itit.value().get<std::string>());
      		int child_id;
      		while(stream >> child_id) {
						road_node->AddLane(GetNodeWithIndex(child_id));
      		}
				}
				else if(!itit.key().compare("label")) {
    		  std::string label_name = itit.value().get<std::string>();
    		  Label* label = grammar_->GetLabelByString(label_name);
    		  if (!label) {
    		    label = new Label(grammar_);
    		    label->SetBasicCategory(label_name);
    		  }
					road_node->SetAssociatedLabel(label);
				}
			}
    }
		else if (!it.key().compare(0, 5, "scene")) {
  		std::cout << "scene" << std::endl;
			// root node 
			for (json::iterator itit = it->begin(); itit != it->end(); ++itit) {
        if(!itit.key().compare("id")) {
    	    id = itit.value().get<int>();
    	  }		
			}
      SceneNode* scene_node = new SceneNode(this, id);
      for (json::iterator itit = it->begin(); itit != it->end(); ++itit) {
        if(!itit.key().compare("children")) {
     			std::stringstream stream(itit.value().get<std::string>());
      		int child_id;
      		while(stream >> child_id) {
						scene_node->AddRoad(GetNodeWithIndex(child_id));
      		}
				}
				else if(!itit.key().compare("label")) {
    		  std::string label_name = itit.value().get<std::string>();
    		  Label* label = grammar_->GetLabelByString(label_name);
    		  if (!label) {
    		    label = new Label(grammar_);
    		    label->SetBasicCategory(label_name);
    		  }
					scene_node->SetAssociatedLabel(label);
				}
				else if(!itit.key().compare("root")) {
		 			if (!grammar_) continue;
		      SetRoot(scene_node);
		   	}
			}
    }
  }
  std::cout << "imported a graph " << filename << std::endl;
}

GraphNode* Graph::Root(void) const {
  return root_;
}

void Graph::SetRoot(GraphNode* pNode) {
	assert(pNode->MyGraph() == this);
	root_ = pNode;
}

GraphNode* Graph::Node(int k) const {
  if(k<0 || k > NNodes()) {
    std::cout << "return a null node" << std::endl;
    return NULL;
  }
  return nodes_.at(k);
}

GraphNode* Graph::GetNodeWithIndex(int k) const {
  if (nodes_map_.find(k) != nodes_map_.end()) {
    return nodes_map_.at(k);
  }
  else {
    std::cout << "return a null node from GetNodeWithIndex" << std::endl;
    return NULL;
  }
}

void Graph::EraseNode(const GraphNode* pNode) {
	std::vector<GraphNode*>::iterator it = std::find(nodes_.begin(), nodes_.end(), pNode);
	if (it != nodes_.end()) {
		delete *it;
		nodes_.erase(it);
	}
	else {
		utility::PrintError("graph is trying to delete a non-existing node", pNode->ToString());
	}
	nodes_map_.erase(pNode->GraphIndex());
}

void Graph::EraseNodePointer(const GraphNode* pNode) {
	std::vector<GraphNode*>::iterator it = std::find(nodes_.begin(), nodes_.end(), pNode);
	if (it != nodes_.end()) {
		nodes_.erase(it);
	}
	else {
		utility::PrintError("graph is trying to erase a non-existing pointer to node", pNode->ToString());
	}
}

void Graph::EraseMarkingNode(const MarkingNode* pNode) {
	std::vector<MarkingNode*>::iterator it = std::find(markings_.begin(), markings_.end(), pNode);
	if (it != markings_.end()) {
		markings_.erase(it);
	}
  else {
		utility::PrintError("graph is trying to delete a non-existing marking", pNode->ToString());
	}
	//EraseNode(pNode);
}

void Graph::EraseBoundaryNode(const BoundaryNode* pNode) {
	std::vector<BoundaryNode*>::iterator it = std::find(boundaries_.begin(), boundaries_.end(), pNode);
	if (it != boundaries_.end()) {
		boundaries_.erase(it);
	}
  else {
		utility::PrintError("graph is trying to delete a non-existing boundary", pNode->ToString());
	}
	//EraseNode(pNode);
}

void Graph::EraseLaneNode(const LaneNode* pNode) {
	std::vector<LaneNode*>::iterator it = std::find(lanes_.begin(), lanes_.end(), pNode);
	if (it != lanes_.end()) {
		lanes_.erase(it);
	}
  else {
		utility::PrintError("graph is trying to delete a non-existing lane", pNode->ToString());
	}
	EraseNode(pNode);
}
  
void Graph::EraseRoadNode(const RoadNode* pNode) {
	std::vector<RoadNode*>::iterator it = std::find(roads_.begin(), roads_.end(), pNode);
	if (it != roads_.end()) {
		roads_.erase(it);
	}
  else {
		utility::PrintError("graph is trying to delete a non-existing road", pNode->ToString());
	}
	EraseNode(pNode);
}

void Graph::EraseSceneNode(const SceneNode* pNode) {
	std::vector<SceneNode*>::iterator it = std::find(scenes_.begin(), scenes_.end(), pNode);
	if (it != scenes_.end()) {
		scenes_.erase(it);
	}
  else {
		utility::PrintError("graph is trying to delete a non-existing scene", pNode->ToString());
	}
	//EraseNode(pNode);
}

Polyline Graph::GetDriveDirection() const {
	return driving_direction_;
}

int Graph::NNodes(void) const {
  return nodes_.size();
}

int Graph::NRoads() const {
  return n_roads_;
}

void Graph::SetGrammar(Grammar* pGrammar) {
  grammar_ = pGrammar;
}

Grammar* Graph::GetGrammar(void) const {
  if (!grammar_) {
		std::cout << "grammar_ is NULL" << std::endl;
	}
	return grammar_;
}

void Graph::InsertNode(GraphNode* pNode) {
	//assert(!pNode->graph_);
	//pNode->graph_ = this;
	nodes_.push_back(pNode);
  nodes_map_.emplace(pNode->GraphIndex(), pNode);
}

void Graph::InsertMarkingNode(MarkingNode* pNode) {
	markings_.push_back(pNode);
}

void Graph::InsertBoundaryNode(BoundaryNode* pNode) {
	boundaries_.push_back(pNode);
}

void Graph::InsertLaneBoundaryNode(LaneBoundaryNode* pNode) {
	lane_boundaries_.push_back(pNode);
}

void Graph::InsertLaneNode(LaneNode* pNode) {
	lanes_.push_back(pNode);
}

void Graph::InsertRoadNode(RoadNode* pNode) {
	roads_.push_back(pNode);
}

void Graph::InsertSceneNode(SceneNode* pNode) {
	scenes_.push_back(pNode);
}

void Graph::AddGraphBoundary(Polyline polyline) {
	if (graph_boundaries_.size() == 0) {
		graph_boundaries_.push_back(polyline);
	}
	else if (graph_boundaries_.size() == 1) {
		//graph_boundaries_.front().LeftOrRight(polyline);
		LineSegment line_segment1 = graph_boundaries_.front().StraightLine();
		LineSegment line_segment2 = polyline.StraightLine();
		if (utility::DotProduct(line_segment1.second - line_segment1.first, line_segment2.second - line_segment2.first) < 0) {
			std::cout << "graph boundary in different direction. reverse second boundary" << std::endl;
			polyline = polyline.Reverse();
		}
		int orientation1 = utility::TripletOrientation(line_segment1.first, line_segment1.second, line_segment2.first);
		int orientation2 = utility::TripletOrientation(line_segment1.first, line_segment1.second, line_segment2.second);
		if (orientation1 == 1 && orientation2 == 1) {
			graph_boundaries_.push_back(polyline);
		}
		else if (orientation1 == 2 && orientation2 == 2) {
			graph_boundaries_.emplace(graph_boundaries_.begin(), polyline);
		}
		else {
			std::cout << "graph vertex in different side. GenerateDriveDirection will fail" << std::endl;
		}
		GenerateDriveDirection();
	}
}

void Graph::GenerateDriveDirection() {
	if (graph_boundaries_.size() == 2) {
		driving_direction_ = graph_boundaries_.front().GetCenterLine(graph_boundaries_.back());
		graph_width_ = graph_boundaries_.front().DistanceBetweenPolyline(graph_boundaries_.back());
		polygon_ = Polygon(graph_boundaries_.front(), graph_boundaries_.back());
	}
	else {
		std::cout << "graph has " << graph_boundaries_.size() << " boundaries, when generating driving direction" << std::endl;
	}
}

double Graph::GetGraphWidth() const {
	return graph_width_;
}

double Graph::GetGraphWidthAtPoint(const Point point) const {
	return polygon_.GetWidthAtPoint(point);
} 

Polygon Graph::GetPolygon() const {
	return polygon_;
}

int Graph::GraphIndex() const {
	return graph_index_;
}

void Graph::SetGraphIndex(const int index) {
	graph_index_ = index;
}

double Graph::SceneSize() const {
	return scene_size_;
}

void Graph::SetSceneSize(double size) {
	scene_size_ = size;
}

std::string Graph::ToString() const {
	return ("Graph" + std::to_string(graph_index_));
}

/*
	
*/
void Graph::SplitBoundary(const LineSegment counter_line) {
	std::vector<Point> points;
	//std::vector<BoundaryNode*> new_boundary_nodes;
	int counter = 0;
	int n_boundary = boundaries_.size();
	//for (auto boundary_node: boundaries_) {
	for (int i = 0; i < n_boundary; i++) {
		BoundaryNode* boundary_node = boundaries_[i];
		std::cout << std::endl << counter <<  " boundary" << std::endl;
		counter ++;
		Polyline polyline = boundary_node->GetPolyline();
		std::vector<LineSegment> line_segments = polyline.GetLineSegments();
		Point point(0, 0);
		bool intersect = false;
		std::pair<Polyline, Polyline> polylines;
		Label* label = grammar_->GetLabelByString("lane_boundary");
		Label* pre_label = grammar_->GetLabelByString("lane");
		Rule* rule = grammar_->GetRulesDerivedFromLabel(pre_label).front();
		for (auto l: line_segments) {
			if(IntersectionPointOfTwoLineSegment(l, counter_line, point)) {
				intersect = true;
				points.push_back(point);
				//std::cout << "IntersectionPointOfTwoLineSegment point is " << point.x <<" " << point.y << std::endl;
			}
		}
		// TODO: Logic is not clear
		if (intersect) {
			//std::cout << " boundary_node->Split(point) begin " << std::endl;
			polylines = boundary_node->Split(point);
			//std::cout << " boundary_node->Split(point) ends" << std::endl;
		}
		else {
			// append a same lane boundary node to the lane boundary 
  		LaneBoundaryNode* lane_boundary_node = new LaneBoundaryNode(this, NNodes(), polyline);
			lane_boundary_node->InsertChild(boundary_node);
			lane_boundary_node->SetAssociatedLabel(label);
      lane_boundary_node->SetAssociatedRule(rule);    
			//lane_boundaries_.push_back(lane_boundary_node);
			continue;
		}
		if (polylines.second.NPoints() == 0) {
			//std::cout << "after splitting, only one polyline!!!!!!" << std::endl;
  		LaneBoundaryNode* lane_boundary_node = new LaneBoundaryNode(this, NNodes(), polylines.first);
			lane_boundary_node->SetAssociatedLabel(label);
      lane_boundary_node->SetAssociatedRule(rule);
			lane_boundary_node->InsertChild(boundary_node);
			//lane_boundaries_.push_back(lane_boundary_node);
		}
		else {
			//std::cout << "after splitting, lines length are" << polylines.first.GetLength() << " " << polylines.second.GetLength() << std::endl;
  		if (polylines.first.GetLength() > 1.0) {
				LaneBoundaryNode* lane_boundary_node1 = new LaneBoundaryNode(this, NNodes(), polylines.first);
				lane_boundary_node1->SetAssociatedLabel(label);
	      lane_boundary_node1->SetAssociatedRule(rule);    
				lane_boundary_node1->InsertChild(boundary_node);
				//lane_boundaries_.push_back(lane_boundary_node1);
			}
			if (polylines.second.GetLength() > 1.0) {
  			LaneBoundaryNode* lane_boundary_node2 = new LaneBoundaryNode(this, NNodes(), polylines.second);
				lane_boundary_node2->SetAssociatedLabel(label);
      	lane_boundary_node2->SetAssociatedRule(rule);    
				lane_boundary_node2->InsertChild(boundary_node);
				if (polylines.first.GetLength() > 1.0) {
					lane_boundary_node2->AddSuccessor(lane_boundaries_.back());
				}
			}
		}
	}
	//lane_boundaries_.insert(lane_boundaries_.end(), lane_boundaries_.begin(), lane_boundaries_.end());
	//return lane_boundaries_;
}

int Graph::NIntersectWithLine(LineSegment line_segment) {
	int count = 0;
	for (auto boundary_node: boundaries_) {
		if (boundary_node->IsIntersect(line_segment)) {
			count ++;
		}
		//std::cout << "NIntersectWithLine count" << count << std::endl;
	}
	return count;
}

bool Graph::IsBoundariesIntersect() const {
	if (boundaries_.size() == 0 || boundaries_.size() == 1) return false;
	for (std::vector<BoundaryNode*>::const_iterator out_it = boundaries_.begin(); out_it != boundaries_.end(); out_it ++) {
		for (std::vector<BoundaryNode*>::const_iterator inner_it = out_it + 1; inner_it != boundaries_.end(); inner_it ++) {
			if ((*out_it)->IsIntersect(*inner_it)) {
				return true;
			}
		}
	}
	return false;
}

int Graph::GenerateLaneBoundary() {
	std::cout << " SplitLane " << driving_direction_.GetLength() << std::endl;
	std::cout << " n boundary nodes in graph " << boundaries_.size() << std::endl;
	std::vector<LineSegment> line_segments = driving_direction_.GetLineSegments();

	int n_intersection = 0;
	int n_split = 0;
	double begin_length = 0.0;
	double end_length = driving_direction_.GetLength();
	for (auto line_segment: line_segments) {
		// point1 point2 are on the driving direction polyline
		Point point1 = line_segment.first;
		Point point2 = line_segment.second;
		// point3 point4 form a perpendicular line to (point1, poin2)
		Point point3;
		Point point4;
		double dx = point2.x - point1.x;
		double dy = point2.y - point1.y;
		Point vector(dx, dy);
		Point basis_vector(dx / vector.Distance(Point(0, 0)), dy / vector.Distance(Point(0, 0)));
		Point orthonomal_vector(-dy / vector.Distance(Point(0, 0)), dx / vector.Distance(Point(0, 0)));
		// split the line segment once per meter
		int n_slice = point1.Distance(point2);
		double line_segment_length = line_segment.first.Distance(line_segment.second);
		double length_per_slice = line_segment_length / n_slice;
		// std::cout << "New line from center line legnth " << line_segment_length << " point1 "<< line_segment_length << " "<< point1.x <<" " <<point1.y << " point2 " << point2.x << " " << point2.y << std::endl;
		// std::cout << " n_slice " << n_slice << std::endl<< std::endl;
		// std::cout << " boundaries " << boundaries_.size() << std::endl<< std::endl;
		for (int i = 0; i < n_slice; i++) {
			end_length -= length_per_slice;
			begin_length += length_per_slice;
			if (begin_length < 15.0) {
				//std::cout << "skipping polyline start " << begin_length << std::endl;				
				continue;
			}
			if (end_length < 15.0) {
				//std::cout << "skipping polyline end " << end_length << std::endl;				
				continue;
			}
			//std::cout << "new counter line" << std::endl;
			point3 = point1 + orthonomal_vector * 50.0 + basis_vector * i;
			point4 = point1 - orthonomal_vector * 50.0 + basis_vector * i;
			LineSegment orthonomal_line(point3, point4);
			std::vector<Point> points;
			int local = NIntersectWithLine(orthonomal_line);
			if (n_intersection != local) {
				if (n_intersection == 0){
					n_intersection = local;
					continue;
				}
				n_split ++;
				// n_intersection changes
				std::cout << std::endl << i << " split" << std::endl;;
				std::cout << "n_intersection " << n_intersection << " local intersection " <<local<<  std::endl;;

				//std::vector<BoundaryNode*> local_new_nodes = SplitBoundary(orthonomal_line);
				SplitBoundary(orthonomal_line);
				//new_boundary_nodes.insert(new_boundary_nodes.begin(), local_new_nodes.begin(), local_new_nodes.end());
				//std::cout << "split line: local " << local <<" n_intersection" << n_intersection << std::endl;;
				n_intersection = local;
				
				//std::cout << "point3 " << point3.x <<" " <<point3.y << " point4 " << point4.x << " " << point4.y << std::endl;
				//std::cout << "graph " << GraphIndex() << " intersects with line "; 
				// for (auto p: points) {
				// 	std::cout << " intersection point " << p.x <<" " << p.y;
				// }
				//std::cout << std::endl;
			}
		}
	}
	// if no split happened, generate lane boundaries same as boundaries
	if (lane_boundaries_.size() == 0) {
		utility::PrintWarning("lane boundary is empty. copy boundaries");
		for (auto boundary_node: boundaries_) {
			Polyline polyline = boundary_node->GetPolyline();
			Label* label = grammar_->GetLabelByString("lane_boundary");
			Label* pre_label = grammar_->GetLabelByString("lane");
			Rule* rule = grammar_->GetRulesDerivedFromLabel(pre_label).front();
  		LaneBoundaryNode* lane_boundary_node = new LaneBoundaryNode(this, NNodes(), polyline);
			lane_boundary_node->InsertChild(boundary_node);
			lane_boundary_node->SetAssociatedLabel(label);
      lane_boundary_node->SetAssociatedRule(rule);    
		}
	}
	n_roads_ = n_split + 1;
	int index = 0;
	for (auto new_boundary: lane_boundaries_) {
		index ++;
		Polyline polyline = new_boundary->GetPolyline();
		//std::cout <<"polyline " << index << " POINTS " << polyline.NPoints() ;
		if (polyline.NPoints() == 0 ) {
			std::cout <<std::endl; 
			continue;
		} 
		std::vector<Point> points = polyline.GetPoints();
		// std::cout <<" points " << points.front().x << " " << points.front().y;
		// std::cout <<" points " << points.back().x << " " << points.back().y << std::endl;
	}
	std::cout <<"boundary size is " << boundaries_.size() << std::endl;
	std::cout <<"lane boundary size is " << lane_boundaries_.size() << std::endl;
	return n_split;
}

void Graph::ProduceLabeledData(const std::string& dirname, int i, json& json_graph) {
  std::string filename = dirname + std::to_string(i) + "/auto_labeled.json";
	Label* label = new Label(grammar_);
	label->SetBasicCategory("lane_boundary");
	int n_road = GenerateLaneBoundary() + 1;
	
	std::cout << markings_.size() << " markings" << std::endl; 
  std::cout << boundaries_.size() << " boundaries" << std::endl; 
  std::cout << lane_boundaries_.size() << " lane_boundaries_" << std::endl; 
  std::cout << n_road << " n_road" << std::endl; 

	for (auto boundary: lane_boundaries_) {
		std::vector<LaneBoundaryNode*> right_boundaries;

		for (auto sibling_boundary: lane_boundaries_) {
			if (boundary == sibling_boundary) {
				continue;
			}
			Point sibling_boundary_point = sibling_boundary->GetPolyline().GetMiddlePoint();
			//std::vector<Point> points = sibling_boundary->GetPolyline().GetPoints();
			//Point sibling_boundary_point = *(points.begin() + points.size() / 2);
			int position = boundary->GetPolyline().PointLeftOrRight(sibling_boundary_point);
			//int position = boundary->Position(sibling_boundary);
			if (position == 1) {
				right_boundaries.push_back(sibling_boundary);
			}
			else if (position == 2 || position == 5) {
				// left_boundary
			}
			else {
  			utility::PrintError("one bad boundary ProduceLabeledData, position code:", position); 
			}
		}
		if (boundary->Child(0)->NChildren() == 0 && right_boundaries.size() < 3) {
			continue;
		}
		if (right_boundaries.size() != 0) {
			std::vector<double> distance_array;
			for (auto right_boundary: right_boundaries) {
				//double distance = boundary->GetPolyline().DistanceBetweenPolyline(right_boundary->GetPolyline());
				Point point1 = boundary->GetPolyline().GetMiddlePoint();
				Point point2 = right_boundary->GetPolyline().GetMiddlePoint();
				double distance = point1.Distance(point2);
				//double distance = boundary->GetPolyline().DistanceTPoint(point2);
				distance_array.push_back(distance);
			}
			int index = std::distance(distance_array.begin(), std::min_element(distance_array.begin(), distance_array.end()));
			LaneNode* lane_node = new LaneNode(this, NNodes());
			lane_node->AddBoundary(boundary);
			lane_node->AddBoundary(right_boundaries[index]);
			// lane_node->InsertChild(boundary);
			// lane_node->InsertChild(right_boundaries[index]);
			std::string label_name = "lane";
    	Label* label = grammar_->GetLabelByString(label_name);
    	if (!label) {
    	  label = new Label(grammar_);
    	  label->SetBasicCategory(label_name);
    	}
			lane_node->SetAssociatedLabel(label);
  		std::cout << lane_node->ToString() << " has children " << boundary->ToString()<<" " << right_boundaries[index]->ToString();
  		std::cout << " Tow boundary distance " << distance_array.at(index) << std::endl;
			// TODO add successors in lane
			for (auto lane: lanes_) {
				LaneBoundaryNode* lane_boundary1 = dynamic_cast<LaneBoundaryNode*>(lane->Child(0));
				LaneBoundaryNode* lane_boundary2 = dynamic_cast<LaneBoundaryNode*>(lane->Child(1));
				Point lane_center_point = lane_node->GetCenterPoint();
				if (lane_boundary1->IsSuccessor(boundary)) {
					int side1 = lane_boundary1->GetPolyline().PointLeftOrRight(lane_center_point);
					int side2 = boundary->GetPolyline().PointLeftOrRight(lane_center_point);
					// if lane boundaries are the same boundary and lane center points are on the same side of this lane boundary
				}
			}
			//lanes_.push_back(lane_node);
		}
	}
	std::cout << "check boundary coordinates " << std::endl;
	for (auto lane_boundary: lane_boundaries_){
		Polyline polyline = lane_boundary->GetPolyline();
		Point point = polyline.GetCenterPoint();
		GraphNode* child = lane_boundary->Child(0);
		std::cout << lane_boundary->ToString() << " x " << point.x << " y " << point.y << " " << child->GraphIndex() << std::endl;

	}
	
	for (std::vector<LaneNode*>::iterator it = lanes_.begin(); it != lanes_.end(); it++) {
		for (std::vector<LaneNode*>::iterator itit = lanes_.begin(); itit != lanes_.end(); itit++) {
			if ((*it)->CheckAddSibling(*itit)) {
				std::cout << (*it)->GraphIndex() << " and " << (*itit)->GraphIndex() << " are siblings" << std::endl;
			}
		}
	}
	//road
  std::cout << markings_.size() << " markings" << std::endl; 
  std::cout << boundaries_.size() << " boundaries" << std::endl; 
  std::cout << lane_boundaries_.size() << " lane_boundaries" << std::endl; 
  std::cout << lanes_.size() << " lanes" << std::endl; 

	std::cout << "before generating road, check the siblings in lanes level " << std::endl;
	for (auto lane1: lanes_) {
		for (auto lane2: lanes_) {
			if (lane2->IsSibling(lane1)) {
				std::cout << lane1->GraphIndex() << " and " << lane2->GraphIndex() << " are siblings" << std::endl;
			}
		}
	}

	std::vector<std::vector<LaneNode*> > road_lane_array;
	std::vector<LaneNode*> added_lane_array;
	road_lane_array.resize(n_road);
	// road_lane_array[0].push_back(lanes_.front());
	// added_lane_array.push_back(lanes_.front());
	// for (auto lane: lanes_) {
	// 	if (std::find(added_lane_array.begin(), added_lane_array.end(), lane) != added_lane_array.end()) {
	// 		continue;
	// 	}
	// 	if (lanes_.front()->IsSibling(lane)) {
	// 		road_lane_array[0].push_back(lane);
	// 		added_lane_array.push_back(lanes_.front());
	// 	}
	// }
	int road_lane_index = 0;
	for (auto lane: lanes_) {
		if (std::find(added_lane_array.begin(), added_lane_array.end(), lane) != added_lane_array.end()) {
			continue;
		}
		std::cout << "2 " << road_lane_index << lane->ToString() << " added_lane_array size " << added_lane_array.size() << std::endl;
		road_lane_array[road_lane_index].push_back(lane);
		added_lane_array.push_back(lane);
		std::cout << "3 " << road_lane_index << lane->ToString() << " added_lane_array size " << added_lane_array.size() << std::endl;
		for (auto lane2: lanes_) {
			if (std::find(added_lane_array.begin(), added_lane_array.end(), lane2) == added_lane_array.end()) {
				if (lane2->IsSibling(lane)) {
					std::cout << "4 " << lane2->ToString() << std::endl;
					road_lane_array[road_lane_index].push_back(lane2);
					added_lane_array.push_back(lane2);
				}
			}
		}
		road_lane_index++;
	}
	std::cout << "4 " << std::endl;

  int index = 0;
	int offset = NNodes();
	for (auto lane_boundary: lane_boundaries_) {
		json points_json = json::array();
		std::vector<Point> points = lane_boundary->GetPolyline().GetPoints();
		for (auto point: points) {
			points_json.push_back(json::object({{"x", std::to_string(point.x + offset_x_)}, {"y", std::to_string(point.y + offset_y_)}}) );
		}
		std::string child_index = std::to_string(lane_boundary->Child(0)->GraphIndex());
    json_graph["n_lane_boundary" + std::to_string(index)]= {
      {"label", "lane_boundary"},
      {"id",  lane_boundary->GraphIndex()},
      {"children", child_index},
			{"points", points_json}
    };
    index += 1;
	}
  index = 0;
  for(auto lane: lanes_) {
		std::string children_indice = std::to_string(lane->Child(0)->GraphIndex())  + " " + std::to_string(lane->Child(1)->GraphIndex());
    json_graph["road_lane" + std::to_string(index)]= {
      {"label", "lane"},
      {"id",  lane->GraphIndex()},
      {"children", children_indice}
    };
    index += 1;
  }
	std::string road_indice;
	for (int i = 0; i < n_road; i++) {
		std::string lane_indice;
		for (auto lane_node: road_lane_array[i]) {
			lane_indice += std::to_string(lane_node->GraphIndex())  + " ";
		}
  	json_graph["road_road" + std::to_string(i)]= {
  	  {"label", "road"},
  	  {"id", NNodes() + i},
  	  {"children", lane_indice}
  	};
		road_indice += std::to_string(NNodes() + i)  + " ";
	}
	json_graph["scene"]= {
  	{"label", "scene"},
  	{"id", NNodes() + n_road},
  	{"children", road_indice},
		{"root", 0}
  };
  //std::ofstream output(dirname + "/labeled.json");
  std::ofstream output(dirname + std::to_string(i) + "auto_labeled.json");
  output << std::setw(4) << json_graph << std::endl;
  utility::PrintGreen("generated labeld file ");
  std::cout << roads_.size() << " n_road" << std::endl; 
	Visualization visualizer(this);
	std::cout << lanes_.size() << " lanes_" << std::endl; 
	cv::Mat lane_image =  visualizer.DrawLanes(this);
  std::cout << lane_boundaries_.size() << " lane_boundaries_" << std::endl; 
	cv::Mat lane_boudnary_image =  visualizer.DrawLaneBoundary(this);
  std::cout << boundaries_.size() << " boundaries" << std::endl; 
	cv::Mat boudnary_image =  visualizer.DrawBoundary(this);
	cv::Mat marking_image = visualizer.DrawMarkings();
	imwrite(std::to_string(i) + "_labeled_lanes.png", lane_image);
	imwrite(std::to_string(i) + "_labeled_lane_boundary.png", lane_boudnary_image);
	imwrite(std::to_string(i) + "_labeled_boundary.png", boudnary_image);
	imwrite(std::to_string(i) + "_labeled_marking.png", marking_image);
}

json Graph::ImportUnlabeledGraph(const std::string& filename, int i) {
	graph_index_ = i;
  std::cout << "begin importing a unlabeled graph " << filename << std::endl;
  
	std::ifstream input;
	input.exceptions(std::ifstream::badbit | std::ifstream::failbit);
  try {
    input.open (filename);
  }
  catch (std::ifstream::failure e) {
    std::cerr << "Exception opening file " << filename << std::endl << std::endl;
  }
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
  offset_x_ = *std::min_element(array_x.begin(), array_x.end()) - 10; 
  offset_y_ = *std::min_element(array_y.begin(), array_y.end()) - 10;
  double scene_width = *std::max_element(array_x.begin(), array_x.end()) - offset_x_;
  double scene_height = *std::max_element(array_y.begin(), array_y.end()) - offset_y_;
  scene_size_ = std::max(scene_width, scene_height);

  //std::cout << " scene_width " << scene_width << " scene_height "<< scene_height << std::endl; 
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
    	    center.x = itit.value().get<double>() - offset_x_;
    	    // std::cout << "center.x  " << center.x << std::endl; 
    	  }
    	  else if(!itit.key().compare("center_y")) {
    	    center.y = itit.value().get<double>() - offset_y_;
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
    		  if (!label) {
    		    label = new Label(grammar_);
    		    label->SetBasicCategory(label_name);
    		  }
				}
			}
    	if (size.width > size.height) {
    	  std::cout << "height width angle before" << size.height << " " << size.width <<" "<< angle << std::endl;
    	  float width= size.height;
    	  size.height = size.width;
    	  size.width = width;
    	  angle += 90.0;
    	}
    	std::cout << "height width angle after" << size.height << " " << size.width <<" "<< angle << std::endl;

    	cv::RotatedRect cv_rect(center, size, angle);
			MarkingNode* marking_node = new MarkingNode(this, id, cv_rect);
			//marking_node->SetBox(cv_rect);
			marking_node->SetAssociatedLabel(label);
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
    		        x = coordinate_it.value().get<double>() - offset_x_;
    		      }
    		      else if(!coordinate_it.key().compare("y")) {
    		        y = coordinate_it.value().get<double>() - offset_y_;
    		      }
    		    }
    		    points.emplace_back(x, y);
    		  }
    		}
				else if(!itit.key().compare("id")) {
    	    id = itit.value().get<int>();
    	  }
				else if(!itit.key().compare("label")) {
    		  std::string label_name = itit.value().get<std::string>();
					if (label_name == "marking_polyline") {
						label_name = "marking_boundary";
					}
    		  label = grammar_->GetLabelByString(label_name);
    		  if (!label) {
    		    label = new Label(grammar_);
    		    label->SetBasicCategory(label_name);
    		  }
				}
			}
			Polyline polyline(points);
			BoundaryNode* boundary_node = new BoundaryNode(this, id, polyline);
			boundary_node->SetAssociatedLabel(label);

			AddGraphBoundary(polyline);
    }
		else if (!it.key().compare(0, 16, "marking_boundary")) {
			// marking boundary node
			for (json::iterator itit = it->begin(); itit != it->end(); ++itit) {
        if(!itit.key().compare("id")) {
    	    id = itit.value().get<int>();
    	  }
			}
			BoundaryNode* boundary_node = new BoundaryNode(this, id);
      for (json::iterator itit = it->begin(); itit != it->end(); ++itit) {
        if(!itit.key().compare("children")) {
					// marking boundary
     			std::stringstream stream(itit.value().get<std::string>());
      		int child_id;
      		while(stream >> child_id) {
						boundary_node->AddMarking(GetNodeWithIndex(child_id));
      		}
				}
				else if(!itit.key().compare("label")) {
    		  std::string label_name = itit.value().get<std::string>();
    		  Label* label = grammar_->GetLabelByString(label_name);
    		  if (!label) {
    		    label = new Label(grammar_);
    		    label->SetBasicCategory(label_name);
    		  }
					boundary_node->SetAssociatedLabel(label);
				}
			}
		}
		json_graph.erase("road_road");
  }
  std::cout << "imported an unlabeled graph " << filename << " NNodes" << NNodes() << std::endl;
	return json_graph;
}

std::vector<MarkingNode*> Graph::GetMarkings() const {
	return markings_;
}

std::vector<BoundaryNode*> Graph::GetBoundaries() const {
	return boundaries_;
}

int Graph::NBoundaries() const {
	return boundaries_.size();
}

std::vector<LaneBoundaryNode*> Graph::GetLaneBoundaries() const {
	return lane_boundaries_;
}

std::vector<LaneNode*> Graph::GetLanes() const {
	return lanes_;
}

std::vector<RoadNode*> Graph::GetRoads() const {
	return roads_;
}

std::vector<SceneNode*> Graph::GetScenes() const {
	return scenes_;
}

void Graph::ExportMarkingEdgeData(std::ofstream& file) const {
	/*
    describe vector for marking node edge is 1.ego.height 2.ego.width 3.other.height 4.other.width 5.dx 6.dy 7.angle 8.sibling
  */
  std::cout << "ExportEdgeData graph_index " << graph_index_ << std::endl;
	int n_edge = 0;
	int n_non_edge = 0;
	for (int i = 0; i < markings_.size(); i ++) {
		MarkingNode* marking_node1 = markings_[i];
		Box box1 =  markings_[i]->GetBox();
		for (int j = 0; j < markings_.size(); j ++) {
			if (i == j) continue;
  		MarkingNode* marking_node2 = markings_[j];
			if (marking_node1->ParentNode() == marking_node2->ParentNode()) {
				file << "1,0,";
				n_edge ++;
			}
			else {
				file << "0,1,";
				n_non_edge ++;
			}
			Box box2 =  markings_[j]->GetBox();
			std::vector<double> relation = box1.GetRelation(box2);

			// try other relation!!!!!!!!!!!!!
			relation = marking_node1->Relation(marking_node2);
			// for (auto re: relation) {
			// 	file << re << ",";
			// }
			for (int k = 0; k < relation.size() - 1; k ++) {
				file << relation[k] << ",";
			}
			file << relation.back();
		  file << "\n";
		}
	}
  std::cout << n_edge << std::endl;
  std::cout << n_non_edge << std::endl;

}
void Graph::ExportLaneBoundaryEdgeData(std::ofstream& file) const {
	/*
    describe vector for lane boundary edge is the distance array, measuring at every 1/30 between two polylines 
  */
  std::cout << "ExportLaneBoundaryEdgeData graph_index " << graph_index_ << std::endl;
	int n_edge = 0;
	int n_non_edge = 0;
	for (int i = 0; i < lane_boundaries_.size(); i ++) {
		LaneBoundaryNode* node1 = lane_boundaries_[i];
		Polyline polyline1 = node1->GetPolyline();
		for (int j = 0; j < lane_boundaries_.size(); j ++) {
			if (i == j) continue;
  		LaneBoundaryNode* node2 = lane_boundaries_[j];
  		Polyline polyline2 = node2->GetPolyline();
			if (node1->ParentNode() == node2->ParentNode()) {
				file << "1,";
				n_edge ++;
			}
			else {
				file << "0,";
				n_non_edge ++;
			}
			std::vector<double> relation = polyline1.GetRelation(polyline2);
			for (auto re: relation) {
				file << re << ",";
			}
		  file << "\n";
		}
	}
  std::cout << n_edge << std::endl;
  std::cout << n_non_edge << std::endl;
}
} //ns
