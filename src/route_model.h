#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>

class RouteModel : public Model {

  public:
	//Model::Node provide the x,y coordinates
    class Node : public Model::Node {
      public:
        Node * parent = nullptr;
        float h_value = std::numeric_limits<float>::max();
        float g_value = 0.0;
        bool visited = false;
        std::vector<Node *> neighbors;

		//FindNeighbors() - will populate the $(neighbors)
        void FindNeighbors();
		
		//distance() - return the distance to the other node
        float distance(Node other) const {
            return std::sqrt(std::pow((x - other.x), 2) + std::pow((y - other.y), 2));
        }

		//constructors
        Node(){}
		//$(idx) - index , used to kepp track on the nodes, and the model to which this node belong
		//$(search_model_ - route model
		//$(node)  - existing model node
		//constructor list - initilaise all this variables
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}

      private:
        int index;
        Node * FindNeighbor(std::vector<int> node_indices);
        RouteModel * parent_model = nullptr;
    };

	//constructor -
	//$(xml) = open streat data
    RouteModel(const std::vector<std::byte> &xml);

	//FindClosestNode() find the closest node a, and give it x,y coordinates
    Node &FindClosestNode(float x, float y);
	//SNodes() - return all the nodes in the model
    auto &SNodes() { return m_Nodes; }

	//$(path) - use to store the final path, will be used to rendering later on
    std::vector<Node> path;
    
  private:
	// cretae a map from the nodes to the roads, that they belong to
    void CreateNodeToRoadHashmap();
	// $(node_to_road) - the map
    std::unordered_map<int, std::vector<const Model::Road *>> node_to_road;
	//$(m_Nodes) - all of the nodes
    std::vector<Node> m_Nodes;

};

#endif
