#include "graph.hpp"
#include "adj_vector.hpp"

int main(){
    std::cout << "testing 2" << std::endl;
    //Graph graph;
    std::string file_path = "../test_subjects/text_file.txt";
    //graph.setSize(10);
    //graph.importEDG();
    AdjVector vec(file_path);
    vec.printGraph();
}