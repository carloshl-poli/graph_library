#include "graph.hpp"

//Construtor
Graph::Graph(bool directed): edge(0), directed(directed), size(0)
{
}
//Destrutor
Graph::~Graph()
{
}

void Graph::setSize(int n){
    size = n;
}

void Graph::importEDG(const std::string& path){
    std::ifstream inputFile(path);

    if (!inputFile){
        std::cerr << "Error! Couldn't open graph " << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << "Tentando abrir: " << path << std::endl;
    std::string line;
    while (std::getline(inputFile, line)){
        std::cout << line << std::endl;
    }
    std::cout << "Terminou de ler " << path << std::endl;

    inputFile.close();

}
