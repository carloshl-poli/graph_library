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
    /// @brief helper initiator for weightless graphs
    /// @param line: A string of format "%d %d" that represents an edge of the graph 
    /// @param isDirected: a boolean that indicates if this is a graph (false) or a digraph (true)
    void helper_init(std::string line, bool isDirected);

    /// @brief helper initiator for weighted graphs
    /// @param line A string of format "%d %d %f" that represents an edge of the graph and its weight
    ///        If this edge contains a weight of value = 0.0, it should indicate this and not just omit the weight
    ///        in the string, as this will throw an error
    /// @param isDirected a boolean that indicates if this is a graph (false) or a digraph (true)
    /// @todo Missing Implementation
    void helper_initWeighted(std::string line, bool isDirected);

protected:

    /// @brief A struct that will be used to store store any Data from the vertex
    ///        For now it stores the vertex degree but can be amplified to store more things
    struct Data {
        int degree = 0;

        Data() {}
        friend class Structure;
        friend class AdjMatrix;
        friend class AdjVector;
    };
    
    int vertexAmount;
    int edgeAmount;
    //std::vector<int> degreeVec;
    //std::vector<std::optional<double>> weightVec;
    /// A map of each data of each vertex in the graph/digraph
    std::unordered_map<int, Data> data;
    void initStruct(const std::string &path, bool isDirected, bool isWeighted);
    
public:
    bool hasWeight;
    std::unordered_map<int, Data> getDataArray();

    //Structure::Structure();
    //Structure(const std::string &path, bool isDirected, bool isWeighted);
    Structure() = default;

    virtual ~Structure() = default;
    
    // Returns true if (V,E) in graph
    virtual bool hasEdgeUV(int V, int U) = 0;
    
    // init the structure with size (size) 
    virtual void resize(int size) = 0;
    
    // Insert edge (U,V) in graph with weight w
    virtual void insertEdge(int U,int V,double w = 1.0) = 0;

    // Returns the weight of the edge (U,V)
    // For weightless graphs, default weight is 1.0
    // In case edge (U,V) not in graph, returns std::nullopt
    virtual std::optional<double> getWeightUV(int V, int U) = 0;

    /// @warning Maybe deprecated method. Tests are necessary
    virtual void printGraph() = 0;
    
    /// @deprecated DELETE WHENEVER IS POSSIBLE
    virtual ReturnType getUAdjArray(int U, bool getWeight = false) = 0;
    
    /// @todo find out if this is necessary, if so still needs implementation
    virtual std::unordered_map<std::optional<double>, std::pair<int, int>> getEdgeList();
    
    /// Return a verctor of all adjacent vertexes of U
    /// @todo Change this implementation so it returns a generator of such array
    virtual std::vector<int> getAdjArray(int U) = 0;

    /// Return a vector of all adjacent vertex of U and the weight of the edge that governs them
    /// @todo same as getAdjArray
    /// @note see if, in case of weightless graph, it should return 1.0, an error, false or something else
    virtual std::vector<std::pair<int, double>> getAdjWeightedArray(int U) = 0;

    /// Returns the graph size
    int getVertexAmount();

    /// Returns the amount of edges
    int getEdgeAmount();

    /// Returns degree value of vertex U
    int getUDegree(int U);
    
    // Returns the DegreeVec
    /// @note Test if this could go to PRIVATE or PROTECTED access 
    std::vector<int> getDegreeVec();

    //void updateWeight(std::size_t u, double weight); REMOVE IF POSSIBLE
    //std::optional<double> getUWeight(int U); REMOVE
    
    //std::vector<std::optional<double>> getWeightVec(); TODO: REMOVE THIS
    //std::vector<std::optional<double>> getWeightedDegreeVec(); REMOVE
/*  template<typename T>
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

*/
  
};


#endif