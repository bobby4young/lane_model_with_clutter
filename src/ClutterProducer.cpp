#include <algorithm> 
#include <iostream>
#include <fstream>
#include <vector>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include "ClutterProducer.h"
#include "json.hpp"
#include "Grammar.h"
#include "Graph.h"
#include "GraphNode.h"
#include "Label.h"
#include "Edge.h"
#include "Rule.h"
#include "ObjectShape.h"
#include "Utility.h"
#include "Visualization.h"

#include <iomanip>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <algorithm>
#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using json = nlohmann::json;

namespace lane_model {

void ClutterProducer::SetGrammar(Grammar* grammar) {
  grammar_ = grammar;
  graph_.SetGrammar(grammar);
}

void ClutterProducer::ImportTerminals(const std::string filename) {
  std::cout << "Parser began import" << std::endl;
  //std::string filename = dirname + ".json";

  std::ifstream input(filename);
  //json json_graph_;
  input >> json_graph_;

  std::vector<double> array_x, array_y;
  for (json::iterator it = json_graph_.begin(); it != json_graph_.end(); ++it) {
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
  }
  offset_x_ = *std::min_element(array_x.begin(), array_x.end()) - 10; 
  offset_y_ = *std::min_element(array_y.begin(), array_y.end()) - 10;
  double scene_width = *std::max_element(array_x.begin(), array_x.end()) - offset_x_;
  double scene_height = *std::max_element(array_y.begin(), array_y.end()) - offset_y_;
  graph_.SetSceneSize(std::max(scene_width, scene_height));

  // std::cout << "offset_x " << offset_x << " offset_y " << offset_y << std::endl; 
  std::map<int, std::map<std::string, std::string> > node_blocks;
  size_t count = 0;
  for (json::iterator it = json_graph_.begin(); it != json_graph_.end(); ++it, count++) {
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
    markings_.push_back(marking_node);
  }
	else if (!it.key().compare(0, 16, "marking_boundary")) {
		// marking boundary node
		for (json::iterator itit = it->begin(); itit != it->end(); ++itit) {
      if(!itit.key().compare("id")) {
  	    id = itit.value().get<int>();
  	  }
		}
		BoundaryNode* boundary_node = new BoundaryNode(&graph_, id);
		//boundaries_.push_back(boundary_node);
    for (json::iterator itit = it->begin(); itit != it->end(); ++itit) {
      if(!itit.key().compare("children")) {
   			std::stringstream stream(itit.value().get<std::string>());
    		int child_id;
    		while(stream >> child_id) {
					boundary_node->AddMarking(graph_.GetNodeWithIndex(child_id));
    		}
			}
			else if(!itit.key().compare("label")) {
  		  std::string label_name = itit.value().get<std::string>();
  		  Label* label = grammar_->GetLabelByString(label_name);
				boundary_node->SetAssociatedLabel(label);
			}
		}
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
      boundaries_.push_back(boundary_node);
	    graph_.AddGraphBoundary(polyline);
	  // boundary_node->SetPolyline(polyline);
	  // std::cout << "boundaries push back at points boundary " << boundaries.size() << std::endl;		
    }
  }
  std::cout << "ClutterProducer imported a graph " << filename << std::endl;
}


void ClutterProducer::ProduceDataWithClutter(const std::string& dirname, int i, const int category) {
  std::cout << markings_.size() << " markings" << std::endl; 
  std::cout << boundaries_.size() << " boundaries" << std::endl; 
  int index = markings_.size();
  //int index = 40;
  int n_marking = markings_.size();

  /* initialize random seed: */
  srand (time(NULL));
  for (int i = 0; i < n_marking; i++) {
    Box box = markings_[i]->GetBox();
    double center_x, center_y, angle;
    if (category == 1) {
      int dx, dy;
      double ratio;
      // generate secret number between 1 and 10:
      dx = rand() % 20 - 10;
      dy = rand() % 20 - 10;
      angle = rand() % 90 - 45;
      ratio = double(rand() % 40 + 80) / 100;
      utility::Print("dx", dx);
      utility::Print("dy", dy);
      utility::Print("angle", angle);
      utility::Print("ratio", ratio);
      std::cout << std::endl;
      center_x = box.CenterX() + dx + offset_x_;
      center_y = box.CenterY() + dy + offset_y_;
      utility::PrintWarning("center_x", center_x);
      utility::PrintWarning("offset_x_", offset_x_);

      json_graph_[std::to_string(index)]= {
        {"label", "marking_segment"},
        {"id",  index},
        {"center_x", center_x},
        {"center_y", center_y},
        {"length", box.GetHeight() * ratio},
        {"width", box.GetWidth()},
        {"orientation", box.GetOrientation() + angle},
      };
    }
    else if (category == 2) {
      Polyline driving_direction = graph_.GetDriveDirection();
      Point marking_center = box.Center();
      LineSegment line_segment = driving_direction.ClosestLineSegment(marking_center);
      Point unit_vector = (line_segment.first - line_segment.second) * (1 / line_segment.first.Distance(line_segment.second));
      Point perpendicular_vector(- unit_vector.y, unit_vector.x);
      double dx, dy;
      dx = 2 * perpendicular_vector.x;
      dy = 2 * perpendicular_vector.y;
      angle = 0.0;    
      utility::Print("dx", dx);
      utility::Print("dy", dy);
      center_x = box.CenterX() + dx + offset_x_;
      center_y = box.CenterY() + dy + offset_y_;
      utility::PrintWarning("center_x", center_x);
      utility::PrintWarning("offset_x_", offset_x_);
  
      json_graph_[std::to_string(index)]= {
        {"label", "marking_segment"},
        {"id",  index},
        {"center_x", center_x},
        {"center_y", center_y},
        {"length", box.GetHeight()},
        {"width", box.GetWidth()},
        {"orientation", box.GetOrientation()},
      };    
    }
    else if (category == 4) {
      angle = rand() % 10 - 5;
      utility::Print("angle", angle);
      center_x = box.CenterX() + offset_x_;
      center_y = box.CenterY() + offset_y_;
      json_graph_[std::to_string(markings_[i]->GraphIndex())]= {
        {"label", "marking_segment"},
        {"id",  markings_[i]->GraphIndex()},
        {"center_x", center_x},
        {"center_y", center_y},
        {"length", box.GetHeight()},
        {"width", box.GetWidth()},
        {"orientation", box.GetOrientation() + angle},
      };
    }
    cv::Point2d center_point{ center_x - offset_x_, center_y - offset_y_};
    cv::Size2d size;
    size.height = box.GetHeight();
    size.width = box.GetWidth();
    cv::RotatedRect cv_rect(center_point, size, box.GetOrientation() + angle);
	  MarkingNode* marking_node = new MarkingNode(&graph_, markings_[i]->GraphIndex());
	  marking_node->SetBox(cv_rect);
    markings_.push_back(marking_node);
    index ++;
  }
  // epecial case: category 3 is to remove one marking from each boundary
  if (category == 3) {
    std::vector<BoundaryNode*> boundaries = graph_.GetBoundaries();
    for (auto boundary: boundaries) {
      int delete_index = -1;
      if (boundary->NChildren() < 3) {
        continue;
      }
      std::vector<MarkingNode*> markings;
      std::vector<double> y_coordinates;
      for (auto m: boundary->Children()) {
        MarkingNode* marking = dynamic_cast<MarkingNode*> (m);
        markings.push_back(marking);
        y_coordinates.push_back(marking->GetBox().CenterY());
      }
      std::vector<double>::iterator max_y = std::max_element(y_coordinates.begin(), y_coordinates.end());
      std::vector<double>::iterator min_y = std::min_element(y_coordinates.begin(), y_coordinates.end());
      // assuming at least three marking existing in this boundary
      if (y_coordinates.at(0) > *min_y && y_coordinates.at(0) < *max_y) {
        delete_index = markings.at(0)->GraphIndex();
      } 
      else if (y_coordinates.at(1) > *min_y && y_coordinates.at(1) < *max_y) {
        delete_index = markings.at(1)->GraphIndex();
      }
      else if (y_coordinates.at(2) > *min_y && y_coordinates.at(2) < *max_y) {
        delete_index = markings.at(2)->GraphIndex();
      }

      if (delete_index != -1) {
        json_graph_.erase(std::to_string(delete_index));
      }
      else {
        utility::PrintError("failed to find a marking in the middile of boundary");
      }
    }
  }
  //std::ofstream output(dirname + "/labeled.json");
  //std::ofstream output(dirname + "/input_clutter.json");
  Visualization visualizer(&graph_);
  cv::Mat marking_image = visualizer.DrawMarkings();
  if (category == 1) {
    std::ofstream output(dirname + std::to_string(i) + "clutter.json");
    output << std::setw(4) << json_graph_ << std::endl;
    imwrite(std::to_string(i) + "_clutter.png", marking_image);
  }
  else {
    std::ofstream output(dirname + std::to_string(i) + "clutter" + std::to_string(category)+ ".json");
    output << std::setw(4) << json_graph_ << std::endl;
    imwrite(std::to_string(i) + "_clutter" +  std::to_string(category)+ ".png", marking_image);  
  }

}

} // ns