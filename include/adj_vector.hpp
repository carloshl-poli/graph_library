#ifndef ADJ_VECTOR_HPP
#define ADJ_VECTOR_HPP

#include "structure_base.hpp"
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <sstream>
#include <utility>
#include <stdexcept>
#include <optional>

class AdjVector : public Structure
{
    private:
        std::vector<std::vector<std::pair<int, double>>> body;

        int getEdgeUV(int U, int V);
    protected:
        void resize(int size) override;
        void insertEdge(int u,int V,double w) override;

    public:
        AdjVector(const std::string &path, bool isWeighted, bool isDirected = false);
        ~AdjVector () = default;

        bool hasEdgeUV(int U, int V) override;
        double getWeightUV(int U, int V) override;

        std::vector<int> getAdjArray(int U) override;
        std::vector<std::pair<int, double>> getAdjWeightedArray(int U) override;
};


#endif