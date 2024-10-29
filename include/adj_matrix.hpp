#ifndef ADJ_MATRIX_HPP
#define ADJ_MATRIX_HPP

#include "structure_base.hpp"
#include "utilities.hpp"
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <sstream>
#include <utility>
#include <stdexcept>
#include <optional>
#include <deque>

class AdjMatrix : public Structure {
    private:
        std::vector<std::vector<std::optional<float>>> body;

    protected:
        void resize(int size) override;
        void insertEdge(int u,int V,float w) override;

    public:
        AdjMatrix(const std::string &path, bool isWeighted, bool isDirected = false);
        ~AdjMatrix () = default;

        bool hasEdgeUV(int U, int V) override;
        float getWeightUV(int U, int V) override;

        std::vector<int> getAdjArray(int U) override;
        std::deque<std::pair<int, float>> getAdjWeightedArray(int U) override;

};





#endif