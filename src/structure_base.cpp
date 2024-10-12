#include "structure_base.hpp"

int Structure::getEdgeAmount(){
    return vertexAmount;
}

int Structure::getVertexAmount(){
    return edgeAmount;
}

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


int Structure::getUDegree(int U){
    return this->degreeVec[U - 1];
}

double Structure::getUWeight(int U){
    return this->weightVec[U - 1];
}

