#ifndef STRUCTURE_BASE_HPP
#define STRUCTURE_BASE_HPP

#include <iostream>
#include <vector>
#include <variant>
#include <utility>
#include <optional>
#include <fstream>
#include <string>
#include <sstream>
#include <stdexcept>
#include <optional>
#include <unordered_map>

using IntVec = std::vector<int>;
using PairVec = std::vector<std::pair<int, std::optional<double>>>;
using ReturnType = std::variant<IntVec, PairVec>;


class AdjMatrix;
class AdjVector;

class Structure {

private:
    void helper_init(std::string line, bool isDirected);
    void helper_initWeighted(std::string line, bool isDirected);

protected:

    struct Data {
        int degree = 0;

        Data() {}
        friend class Structure;
        friend class AdjMatrix;
        friend class AdjVector;
    };
    
    int vertexAmount;
    int edgeAmount;
    std::vector<int> degreeVec;
    std::vector<std::optional<double>> weightVec;
    std::unordered_map<int, Data> data;
    void initStruct(const std::string &path, bool isDirected, bool isWeighted);
    
public:
    bool hasWeight;
    std::unordered_map<int, Data> getDataArray();

    //Structure::Structure();
    //Structure(const std::string &path, bool isDirected, bool isWeighted);
    Structure() = default;

    virtual ~Structure() = default;
    virtual bool hasEdgeUV(int V, int U) = 0;
    virtual void resize(int size) = 0;
    virtual void insertEdge(int U,int V,double w = 1.0) = 0;
    virtual std::optional<double> getWeightUV(int V, int U) = 0;
    virtual void printGraph() = 0;
    virtual ReturnType getUAdjArray(int U, bool getWeight = false) = 0;
    virtual std::unordered_map<std::optional<double>, std::pair<int, int>> getEdgeList();
    virtual std::vector<int> getAdjArray(int U) = 0;
    virtual std::vector<std::pair<int, double>> getAdjWeightedArray(int U) = 0;


    int getVertexAmount();
    int getEdgeAmount();
    int getUDegree(int U);
    void updateWeight(std::size_t u, double weight);
    std::optional<double> getUWeight(int U);
    std::vector<int> getDegreeVec();
    std::vector<std::optional<double>> getWeightVec();
    std::vector<std::optional<double>> getWeightedDegreeVec();

    template<typename T>
    static void setUData(int U, std::vector<T> &vec, T data){
        if (U > 0 && U <= vec.size()) {
            vec[U - 1] = data;
        }
        else {
            throw std::out_of_range("Index U is out of range");
        }
    }

    template<typename T>
    static T Structure::getUData(int U, std::vector<T> &vec){
        if (U > 0 && U <= vec.size()) {
            return vec[U - 1];
        } 
        else {
            throw std::out_of_range("Index U is out of range");
        }
    }


};


#endif