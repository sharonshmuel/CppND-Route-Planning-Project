#include "route_model.h"
#include <iostream>

//definitions

//class constructor
//constructor list - create the model using the xml data
RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    // Create RouteModel nodes.
    int counter = 0;
    for (Model::Node node : this->Nodes()) {
		//emplace_back() - musch like push_back() , returns a reference to the object that just been pushed
        m_Nodes.emplace_back(Node(counter, this, node));
        counter++;
    }
    CreateNodeToRoadHashmap();
}

//CreateNodeToRoadHashmap(): create a map from the index of the node to the road that it belongs to
void RouteModel::CreateNodeToRoadHashmap() {
	//iterate on all the roads in the model
    for (const Model::Road &road : Roads()) {
		//only include roads that we can drive on (waze)
        if (road.type != Model::Road::Type::Footway) {
			// we get the way that the road is belongs to
			// we get all the nodes in that way that the road is belongs to
            for (int node_idx : Ways()[road.way].nodes) {
				//we check if this index is already in the hashmap
                if (node_to_road.find(node_idx) == node_to_road.end()) {
					//if not we create an empty vector
                    node_to_road[node_idx] = std::vector<const Model::Road *> ();
                }
				//we push the road to the the map
                node_to_road[node_idx].push_back(&road);
            }
        }
    }
}

//$(node_indices): take vector of node indices
//return closest node in that vector to the current node
RouteModel::Node *RouteModel::Node::FindNeighbor(std::vector<int> node_indices) {
    Node *closest_node = nullptr;
    Node node;

    for (int node_index : node_indices) {
		//find the node object taht belong to that iindics
        node = parent_model->SNodes()[node_index];
        if (this->distance(node) != 0 && !node.visited) {
            if (closest_node == nullptr || this->distance(node) < this->distance(*closest_node)) {
                closest_node = &parent_model->SNodes()[node_index];
            }
        }
    }
    return closest_node;
}


void RouteModel::Node::FindNeighbors() {
	//iterate over the roads from the node
    for (auto & road : parent_model->node_to_road[this->index]) {
		//closest node from the nodes that belong to this way
        RouteModel::Node *new_neighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
        if (new_neighbor) {
			//we push closest node from each way, if exist one
            this->neighbors.emplace_back(new_neighbor);
        }
    }
}


RouteModel::Node &RouteModel::FindClosestNode(float x, float y) {
    Node input;
    input.x = x;
    input.y = y;

    float min_dist = std::numeric_limits<float>::max();
    float dist;
    int closest_idx;

	//iterate ove all the roads in the map
    for (const Model::Road &road : Roads()) {
		// only drive
        if (road.type != Model::Road::Type::Footway) {
			// iterate over all the nodes , that on the way , by their $(nodes) indices
            for (int node_idx : Ways()[road.way].nodes) {
				//$(dist): calculate distance from current node $(input) to node on the way 
                dist = input.distance(SNodes()[node_idx]);
                if (dist < min_dist) {
                    closest_idx = node_idx;
                    min_dist = dist;
                }
            }
        }
    }
	//return closest distance node
    return SNodes()[closest_idx];
}