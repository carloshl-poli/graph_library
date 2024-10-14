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

class AdjMatrix : public Structure
{
private:
    std::vector<std::vector<std::optional<double>>> body;
public:
    AdjMatrix(const std::string &path, bool isDirected = false);
    ~AdjMatrix ();

    void resize(int size) override;
    void insertEdge(int u,int V,double w) override;
    bool hasEdgeUV(int U, int V) override;
    std::optional<double> getWeightUV(int U, int V) override;
    ReturnType getUAdjArray(int U, bool getWeight = false);
    void printGraph() override;

    std::vector<int> getAdjArray(int U);
    std::vector<std::pair<int, double>> getAdjWeightedArray(int U);

};





#endif