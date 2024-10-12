#include "adj_matrix.hpp"

/// @deprecated Now using Parent's constructor
AdjMatrix::AdjMatrix (const std::string &path, bool isDirected) {
    std::string line;
    std::ifstream graphFile(path);
    this->edgeAmount = 0;
    this->vertexAmount = 0;

    if (!graphFile.is_open()){
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
            this->insertEdge(u,v, weight);
            this->edgeAmount++;
            if (!isDirected){
                this->insertEdge(v,u, weight);
            }
        }

    }
    
}

void AdjMatrix::insertEdge(int u, int v, double w){
    this->body[u][v] = w;
}

void AdjMatrix::resize(int n){
    this->body.resize(n, std::vector<std::optional<double>>(n, std::nullopt));
}

bool AdjMatrix::hasEdgeUV(int U, int V){
    return this->body[U-1][V-1].has_value();
}

std::optional<double> AdjMatrix::getWeightUV(int U, int V){
    if (this->hasEdgeUV(U, V)){
        return this->body[U-1][V-1];
    }
    throw std::out_of_range("Error: There is no edge between vertices "
        + std::to_string(V) + " and " + std::to_string(U) + ".");
}

/// @todo Reescrever para eliminar repetição de código
ReturnType AdjMatrix::getUAdjArray(int U, bool getWeight){
    if (getWeight){
        std::vector<std::pair<int, std::optional<double>>> adjArray;
        for (int v = 0; v < this->body.size(); v++){
            int V = v + 1;
            int u = U - 1;
            if (this->body[u][v].has_value()){
                adjArray.push_back(std::make_pair(V, this->getWeightUV(U, V)));
            }
        }
    }
    else {
        std::vector<int> adjArray;
        for (int v = 0; v < this->body.size(); v++){
            int V = v + 1;
            int u = U - 1;
            if (this->body[u][v].has_value()){
                adjArray.push_back(V);
            }
        }
    }
}

AdjMatrix::~AdjMatrix()
{
}