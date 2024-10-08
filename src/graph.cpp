#include "graph.hpp"

class Graph{
    private:
        int edge;
        bool directed;
        int size;

    public:
        Graph(bool directed = false): directed(directed){
        }

        void setSize(int n){
            this->size = n;
        }

        void importEDG(const std::string& path){
            std::ifstream inputFile(path);

            if (!inputFile){
                std::cerr << "Error! Couldn't open graph" << std::endl;
                exit(EXIT_FAILURE);
            }

            std::string line;
            while (std::getline(inputFile, line)){
                std::cout << line << std::endl;
            }

            inputFile.close();

        }
    
    
};