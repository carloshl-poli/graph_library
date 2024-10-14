#include "structure_base.hpp"

int Structure::getEdgeAmount(){
    return vertexAmount;
}

int Structure::getVertexAmount(){
    return edgeAmount;
}


/*
Structure::Structure(const std::string &path, bool isDirected) {
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
            this->insertEdge(u,V, weight);
            this->edgeAmount++;
            this->degreeVec[u]++;
            this->weightVec[u]++;
            if (!isDirected){
                this->insertEdge(v,U, weight);
                this->degreeVec[v]++;
                this->weightVec[v]++;
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
*/

int Structure::getUDegree(int U){
    return this->degreeVec[U - 1];
}

void Structure::updateWeight(std::size_t u, double weight) {
    if (weightVec[u].has_value()) {
        weightVec[u] = weightVec[u].value() + weight;
    } else {
        weightVec[u] = weight;
    }
}

std::optional<double> Structure::getUWeight(int U){
    return this->weightVec[U - 1];
}

std::vector<int> Structure::getDegreeVec(){
    return this->degreeVec;
}

std::vector<std::optional<double>> Structure::getWeightVec(){
    return this->weightVec;
}

std::vector<std::optional<double>> Structure::getWeightedDegreeVec() {
    std::vector<int> degreeVec = this->getDegreeVec();
    std::vector<std::optional<double>> weightVec = this->getWeightVec();
    std::vector<std::optional<double>> weightedVec(degreeVec.size());
    for (size_t i = 0; i < degreeVec.size(); i++){
        if (weightVec[i]){
            weightedVec[i] = *weightVec[i] * degreeVec[i];
        }
        
    }
    return weightedVec;
}

void Structure::helper_init(std::string line, bool isDirected) {
    std::istringstream iss(line);
    int U, V;
    if (iss >> U >> V){
        if (U < 1 || V < 1 || U > this->vertexAmount || V > this->vertexAmount) {
            throw std::out_of_range("Error! índex out of bounds");
        }
        this->insertEdge(U,V);
        this->edgeAmount++;
        this->data[U].degree++;
        if (!isDirected){
            this->insertEdge(V,U);
            this->data[U].degree++;
        }
    }
    else {
        throw std::invalid_argument("line must be 'int int'");
    }
}

void Structure::initStruct(const std::string &path, bool isDirected, bool isWeighted)
{
    std::string line;
    std::ifstream graphFile(path);
    this->edgeAmount = 0;
    this->vertexAmount = 0;

    if (!graphFile.is_open()){
        throw std::runtime_error("Error! Couldn't open graph file");
    }

    //Set vertex Amount to the first line
    if (getline(graphFile, line)){
        this->vertexAmount = std::stoi(line);
    }
    else {
        throw std::invalid_argument("invalid graph size, must be a single integer");
    }

    this->resize(this->vertexAmount);

    while (getline(graphFile, line)){
        if (isWeighted){
            this->helper_init(line, isDirected);
        }
        else {
            this->helper_initWeighted(line, isDirected);
        }
    }
    graphFile.close();
}

std::unordered_map<int, Structure::Data> Structure::getDataArray()
{
    return this->data;
}
