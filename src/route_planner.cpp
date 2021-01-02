#include "route_planner.h"
#include <algorithm>
using std::sort;

bool Compare(const RouteModel::Node *a, const RouteModel::Node *b);

//class constructor
//initializer : initialize $(m_Model) to refer to $(model)
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
	start_node = &m_Model.FindClosestNode(start_x, start_y);
	end_node = &m_Model.FindClosestNode(end_x, end_y);
	start_node->g_value = 0.0f;
	end_node->h_value = 0.0f;
	//calculate distance to end_node
	start_node->h_value = CalculateHValue(start_node);
	start_node->visited = true;
	open_list.push_back(start_node);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	current_node->FindNeighbors();
	for (auto nn : current_node->neighbors) {
		//set the parent, the h_value, the g_value
		if (!nn->visited) {
			nn->parent = current_node;
			nn->g_value = current_node->g_value + current_node->distance(*nn); 
			nn->h_value = CalculateHValue(nn);
			nn->visited = true;
			open_list.push_back(nn);
		}
	}
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

bool Compare(const RouteModel::Node *a, const RouteModel::Node *b) {
	float f1 = a->g_value + a->h_value; // f1 = g1 + h1
	float f2 = b->g_value + b->h_value; // f2 = g2 + h2
	return f1 > f2;
}

RouteModel::Node *RoutePlanner::NextNode() {
	sort(open_list.begin(), open_list.end(), Compare);
	auto current = open_list.back();
	open_list.pop_back();
	return current;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
	std::vector<RouteModel::Node> path_found;
	
    // TODO: Implement your solution here.
	while (current_node->parent != NULL) {
		distance += current_node->distance(*current_node->parent);
		path_found.emplace(path_found.begin(),*current_node);
		if ((current_node->x==start_node->x) && (current_node->y == start_node->y))
			break;
		if ((current_node->parent->x == start_node->x) && (current_node->parent->y == start_node->y)) {
			path_found.emplace(path_found.begin(), *current_node->parent);
			break;
		}
		current_node = current_node->parent;
	}
    
	std::cout << "\nPath : n " << path_found.size() << "\n";
	for (auto p : path_found)
		std::cout <<"(" << p.x << "," << p.y << ")\n";


	distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
	RouteModel::Node *current_node = nullptr;
#ifdef DBG
	std::cout << "\tAStarSearch(): " << " start node : " << " location : (" << start_node->x << ", " << start_node->y << ") g : " << start_node->g_value << " h : " << start_node->h_value << "\n";
	std::cout << "\tAStarSearch(): " << " end node   : " << " location : (" << end_node->x   << ", " << end_node->y   << ") g : " << end_node->g_value   << " h : " << end_node->h_value   << "\n";
	std::cout << "\n";
#endif
	// TODO: Implement your solution here.
	while (open_list.size() > 0) {
		current_node = NextNode();
#ifdef DBG
		std::cout << "\tcurrent node : " << " location : (" << current_node->x << "," << current_node->y << ") g : " << current_node->g_value << " h : " << current_node->h_value << "\n";
#endif
		if (!((current_node->x == end_node->x) && (current_node->y == end_node->y)))
			AddNeighbors(current_node);
		else 
			break;
	}
	if ((current_node!=NULL) && (current_node->x == end_node->x) && (current_node->y == end_node->y))
		this->m_Model.path = ConstructFinalPath(current_node);
}