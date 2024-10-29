#ifndef UTILITIES_HPP
#define UTILITIES_HPP
#include <windows.h>
#include <psapi.h>
#include <iostream>

void logMemoryUsage(const std::string &context);

void logMemoryUsage(const std::string &context, int edgeCount);

#endif