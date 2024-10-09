#include "graph.hpp"

int main(){
    std::cout << "testing 2" << std::endl;
    Graph graph;

    graph.setSize(10);
    graph.importEDG("../test_subjects/text_file.txt");
}