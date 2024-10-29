#include "utilities.hpp"

void logMemoryUsage(const std::string &context) {
    PROCESS_MEMORY_COUNTERS memInfo;
    if (GetProcessMemoryInfo(GetCurrentProcess(), &memInfo, sizeof(memInfo))) {
        SIZE_T memoryUsedKB = memInfo.WorkingSetSize / (1024 * 1024);
        std::cout << "[" << context << "] Uso de memória: " << memoryUsedKB << " KB\n";
    }
}

void logMemoryUsage(const std::string &context, int edgeCount) {
    PROCESS_MEMORY_COUNTERS memInfo;
    if (GetProcessMemoryInfo(GetCurrentProcess(), &memInfo, sizeof(memInfo))) {
        SIZE_T memoryUsedKB = memInfo.WorkingSetSize / (1024 * 1024);
        std::cout << "[" << context << "] Uso de memória: " << memoryUsedKB << " KB, Arestas inseridas: " << edgeCount << "\n";
    }
}
