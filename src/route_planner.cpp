#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model)
{
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

// CalculateHValue method.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node)
{

    return node->distance(*end_node);
}

// AddNeighbors method :  expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
    //Populate current_node.neighbors vector with all the neighbors
    current_node->FindNeighbors();
    //Set the parent, the h_value, the g_value for the current node neighbors
    for (auto &neighbor : current_node->neighbors)
    {
        neighbor->parent = current_node;
        neighbor->h_value = this->CalculateHValue(neighbor);
        neighbor->g_value = neighbor->distance(*start_node);
        neighbor->visited = true;
        open_list.emplace_back(neighbor);
    }
}

// NextNode method: to sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode()
{
    //Sort nodes according to g_val+h_val in descending order
    struct
    {
        bool operator()(RouteModel::Node *a, RouteModel::Node *b) const
        {

            return a->g_value + a->h_value > b->g_value + b->h_value;
        }
    } fsort;
    std::sort(open_list.begin(), open_list.end(), fsort);
    //Return the lowest sum h_val+g_val node
    RouteModel::Node *last = open_list.back();
    open_list.pop_back();
    return last;
}

// ConstructFinalPath method: return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    
    while(start_node !=current_node){
        path_found.insert(path_found.begin(),*current_node);
        distance = current_node->distance(*current_node->parent);
        distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
        current_node=current_node->parent;
    }    
    path_found.insert(path_found.begin(),*start_node);

    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch()
{
    RouteModel::Node *current_node = nullptr;
    current_node = start_node;
    while(current_node!=end_node){
        AddNeighbors(current_node);
        current_node = NextNode();
    }
   m_Model.path= ConstructFinalPath(end_node);

}