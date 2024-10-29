#ifndef ADJ_VECTOR_HPP
#define ADJ_VECTOR_HPP

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

class AdjVector : public Structure
{
    private:
        std::vector<std::deque<std::pair<int, float>>> body;

        int getEdgeUV(int U, int V);
    protected:
        void resize(int size) override;
        void insertEdge(int u,int V,float w) override;

    public:
        AdjVector(const std::string &path, bool isWeighted, bool isDirected = false);
        ~AdjVector () = default;

        bool hasEdgeUV(int U, int V) override;
        float getWeightUV(int U, int V) override;

        std::vector<int> getAdjArray(int U) override;
        std::deque<std::pair<int, float>> getAdjWeightedArray(int U) override;
};


#endif