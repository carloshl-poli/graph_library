#include "graph.hpp"
#include "adj_vector.hpp"
#include "adj_matrix.hpp"
#include "structure_base.hpp"
#include <iostream>

void printValues(const ReturnType& values) {
    // Usando std::visit para lidar com os diferentes tipos de valores
    std::visit([](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;

        if constexpr (std::is_same_v<T, IntVec>) {
            // Caso seja um vetor de inteiros
            for (const auto& val : arg) {
                std::cout << val << " ";
            }
            std::cout << std::endl;
        } else if constexpr (std::is_same_v<T, PairVec>) {
            // Caso seja um vetor de pares (int, std::optional<double>)
            for (const auto& val : arg) {
                std::cout << "(" << val.first << ", ";
                
                // Verifica se o std::optional<double> contém um valor
                if (val.second.has_value()) {
                    std::cout << val.second.value();  // Imprime o valor de double
                } else {
                    std::cout << "nullopt";  // Imprime nullopt se não houver valor
                }

                std::cout << ") ";
            }
            std::cout << std::endl;
        }
    }, values);
}
/*
int main(){
    std::cout << "testing 2" << std::endl;
    //Graph graph;
    std::string file_path = "../test_subjects/text_file.txt";
    //graph.setSize(10);
    //graph.importEDG();
    AdjMatrix vec(file_path);
    printValues(vec.getUAdjArray(5));
}
*/