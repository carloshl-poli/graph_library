#include "adj_vector.hpp"



AdjVector::AdjVector(const std::string &path, bool isDirected) {
    std::string line;
    std::ifstream graphFile(path);
    this->edgeAmount = 0;
    this->vertexAmount = 0;

    if (!graphFile){
        std::cerr << "Error! Couldn't open graph" << std::endl;
        exit(EXIT_FAILURE);
    }
    //Set vertex Amount to the first line
    if (getline(graphFile, line)){
        this->vertexAmount = std::stoi(line);
    }

    this->resize(this->vertexAmount);


    while (getline(graphFile, line)){
        std::istringstream iss(line);
        int u, v, U, V;
        double weight;

        if (iss >> U >> V){
            u = U - 1;
            v = V - 1;

            if (u < 0 || v < 0 || u >= this->vertexAmount || v >= this->vertexAmount) {
                std::cerr << "Error! índex out of bounds: " << U << ", " << V << std::endl;
                exit(EXIT_FAILURE);
            }

            if (!(iss >> weight)){
                weight = 1.0;
            } 
            this->insertEdge(u,V, weight);
            this->edgeAmount++;
            if (!isDirected){
                this->insertEdge(v,U, weight);
            }
        }

    }
    
}

void AdjVector::insertEdge(int u, int V, double weight) {
    this->body[u].emplace_back(V, weight);
}

void AdjVector::resize(int size){
    this->body.resize(size);
}

int AdjVector::getEdgeUV(int U, int V){
    for (int index = 0; index < this->body[U - 1].size(); index++){
        if (this->body[U-1][index].first == V){
            return index;
        }
    }
    throw std::out_of_range("Edge UV not inside graph");
}

bool AdjVector::hasEdgeUV(int U, int V){
    int index;
    try
    {
        index = this->getEdgeUV(U,V);
        return true;
    }
    catch(const std::out_of_range& e)
    {
        return false;
    }
}
    

double AdjVector::getWeightUV(int U, int V){
    int v = this->getEdgeUV(U,V);
    return this->body[U-1][v].second;
}

void AdjVector::setWeightUV(int U, int V, double newWeight){
    int v = this->getEdgeUV(U,V);
    this->body[U-1][v].second = newWeight;
}

void AdjVector::printGraph() {
    std::cout << "Edge list (u -> v):\n";
    for (int u = 0; u < this->body.size(); u++) {
        for (const auto& edge : this->body[u]) {
            int v = edge.first;
            double weight = edge.second;
            std::cout << (u + 1) << " -> " << v << " [weight: " << weight << "]\n";
        }
    }
}


AdjVector::~AdjVector()
{
}