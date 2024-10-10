#ifndef STRUCTURE_BASE_HPP
#define STRUCTURE_BASE_HPP

#include <iostream>

class Structure {

public:
    Structure(/* args */);
    ~Structure();
    virtual bool hasEdgeUV(int V, int U) = 0;
    
};

Structure::Structure(/* args */)
{
}

Structure::~Structure()
{
}


#endif