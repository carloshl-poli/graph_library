#include "graph.hpp"
#include "adj_vector.hpp"

void printValues(const ReturnType& values) {
    // Usando std::visit para lidar com os diferentes tipos de valores
    std::visit([](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;

        if constexpr (std::is_same_v<T, IntVec>) {
            for (const auto& val : arg) {
                std::cout << val << " ";
            }
            std::cout << std::endl;
        } else if constexpr (std::is_same_v<T, PairVec>) {
            for (const auto& val : arg) {
                std::cout << "(" << val.first << ", " << val.second << ") ";
            }
            std::cout << std::endl;
        }
    }, values);
}

int main(){
    std::cout << "testing 2" << std::endl;
    //Graph graph;
    std::string file_path = "../test_subjects/text_file.txt";
    //graph.setSize(10);
    //graph.importEDG();
    AdjVector vec(file_path);
    printValues(vec.getUAdjArray(5));
}