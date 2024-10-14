#include "adj_matrix.hpp"


AdjMatrix::AdjMatrix(const std::string &path, bool isDirected) {
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
    this->degreeVec.resize(this->vertexAmount, 0);
    this->weightVec.resize(this->vertexAmount, 0);

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
            this->degreeVec[u]++;
            this->updateWeight(u, weight);
            
            if (!isDirected){
                this->insertEdge(v,u, weight);
                this->degreeVec[v]++;
                this->updateWeight(v, weight);
            }
        }

    }
    ///TODO: LIMPAR ESSE LIXO
    graphFile.clear();
    graphFile.seekg(0);
    getline(graphFile, line);
    getline(graphFile, line);
    std::istringstream iss(line);
    int a, b;
    double c;
    if (iss >> a >> b >> c){
        this->hasWeight = true;
    }
    graphFile.close();
}

void AdjMatrix::insertEdge(int U, int V, double w){
    this->body[U-1][V-1] = w;
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

/// @deprecated
/// @todo Verificar possível remoção do método
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
        return adjArray;
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
        return adjArray;
    }

    
}

void AdjMatrix::printGraph() {
    std::cout << "Edge list (u -> v):\n";
    for (int u = 0; u < this->body.size(); u++) {
        for (int v = 0; v < this->body.size(); v++) {
            if (this->body[u][v].has_value()){
                int U = u + 1;
                int V = v + 1;
                std::optional<double> weight = this->body[u][v];
                std::cout << U << " -> " << V << " [weight: " << weight.value() << "]\n";
            }
        }
    }
}

std::vector<int> AdjMatrix::getAdjArray(int U) {
    std::vector<int> adjArray;
    int u = U - 1;
    for (int v = 0; v < this->vertexAmount; v++){
        int V = v + 1;
        if (this->body[u][v].has_value()){
            adjArray.push_back(V);
        }
    }
    return adjArray;
}

std::vector<std::pair<int, double>> AdjMatrix::getAdjWeightedArray(int U) {
    std::vector<std::pair<int, double>> adjWeightedArray;
    int u = U - 1;
    for (int v = 0; v < this->vertexAmount; v++){
        int V = v + 1;
        if (this->body[u][v].has_value()){
            adjWeightedArray.push_back(std::pair<int, double>(V, this->body[u][v].value()));
        }
    }

    return adjWeightedArray;
}

AdjMatrix::~AdjMatrix()
{
}