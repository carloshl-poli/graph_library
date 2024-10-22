#ifndef ADJ_MATRIX_HPP
#define ADJ_MATRIX_HPP

#include "structure_base.hpp"
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <sstream>
#include <utility>
#include <stdexcept>
#include <optional>

class AdjMatrix : public Structure {
    private:
        std::vector<std::vector<std::optional<double>>> body;

    protected:
        void resize(int size) override;
        void insertEdge(int u,int V,double w) override;

    public:
        AdjMatrix(const std::string &path, bool isWeighted, bool isDirected = false);
        ~AdjMatrix () = default;

        bool hasEdgeUV(int U, int V) override;
        double getWeightUV(int U, int V) override;

        std::vector<int> getAdjArray(int U) override;
        std::vector<std::pair<int, double>> getAdjWeightedArray(int U) override;

};





#endif