#include "graph.hpp"

//Construtor
Graph::Graph(std::string &path, GraphStructure structure, bool isDirected){
    switch (structure) {
        case GraphStructure::AdjMatrix:
            this->structure = std::make_unique<AdjMatrix>(path, isDirected);
            break;
        case GraphStructure::AdjVector:
            this->structure = std::make_unique<AdjVector>(path, isDirected);
            break;
        default:
            throw std::out_of_range("Error: Invalid or not supported Structure");
            break;
    }

    this->vertexAmount = this->structure->getVertexAmount();
    this->edgeAmount = this->structure->getEdgeAmount();
    this->hasWeight = this->structure->hasWeight;

}

void Graph::setStructure(GraphStructure structure){
    
}
