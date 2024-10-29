#include "structure_base.hpp"

int Structure::getEdgeAmount(){
    return this->edgeAmount;
}

int Structure::getVertexAmount() const{
    return this->vertexAmount;
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
            this->data[V].degree++;
        }
    }
    else {
        throw std::invalid_argument("line must be 'int int'");
    }
}

void Structure::helper_initWeighted(std::string line, bool isDirected) {
    std::istringstream iss(line);
    int U, V;
    float w;
    if (iss >> U >> V >> w){
        if (U < 1 || V < 1 || U > this->vertexAmount || V > this->vertexAmount) {
            throw std::out_of_range("Error! índex out of bounds");
        }
        this->insertEdge(U, V, w);
        this->edgeAmount++;
        this->data[U].degree++;
        if (!isDirected){
            this->insertEdge(V, U, w);
            this->data[V].degree++;
        }
    }
    else {
        throw std::invalid_argument("line must be 'int int float'");
    }
}

void Structure::initStruct(const std::string &path, bool isDirected, bool isWeighted) {
    logMemoryUsage("Início de initStruct");

    if (isDirected){
        throw std::runtime_error("Error! Directed Graphs aren't currently supported");
    }

    std::string line;
    std::ifstream graphFile(path);
    this->edgeAmount = 0;
    this->vertexAmount = 0;
    this->isWeighted = true ? isWeighted : false;

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

    this->resize(this->getVertexAmount());
    logMemoryUsage("Após redimensionar a estrutura");

    while (getline(graphFile, line)){
        if (!isWeighted){
            this->helper_init(line, isDirected);
        }
        else {
            this->helper_initWeighted(line, isDirected);
        }
    }
    logMemoryUsage("Final de initStruct");
    graphFile.close();
}

std::unordered_map<int, Structure::Data> Structure::getDataArray()
{
    return this->data;
}


int Structure::getUDegree(int U) {
    return data[U].degree;
}

bool Structure::hasWeight() {
    return isWeighted;
}


