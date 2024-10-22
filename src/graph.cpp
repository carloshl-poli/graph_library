#include "graph.hpp"

// Construtor
Graph::Graph(std::string &path, GraphStructure structure, bool isDirected, bool isWeighted){
    switch (structure) {
        case GraphStructure::AdjMatrix:
            this->structure = std::make_unique<AdjMatrix>(path, isWeighted, isDirected);
            break;
        case GraphStructure::AdjVector:
            this->structure = std::make_unique<AdjVector>(path, isWeighted, isDirected);
            break;
        default:
            throw std::invalid_argument("Error: Invalid or not supported Structure");
            break;
    }

    this->vertexAmount = this->structure->getVertexAmount();
    this->edgeAmount = this->structure->getEdgeAmount();
    this->hasWeight = this->structure->hasWeight();

}

double Graph::getMean(){
    auto structData = this->structure->getDataArray();
    if (structData.empty()) return 0.0;

    double degreeSum = 0.0;
    for (size_t v = 0; v < structData.size(); v++){
        degreeSum += structData[v].degree;
    }
    return degreeSum / structData.size();
}

/// @note Ready for tests
double Graph::getMedian() {
    auto structData = this->structure->getDataArray();
    int n = structData.size(); 
        if (n % 2 == 0){

            int medianIndex = n / 2;
            return (structData[medianIndex].degree + structData[medianIndex + 1].degree ) / 2;
        }
        else{
            int medianIndex = std::floor(n / 2);
            return structData[medianIndex].degree; 
        }
}

int Graph::getMinDegree() {
    int minDegree = std::numeric_limits<int>::max();
    auto structData = this->structure->getDataArray();

    for (const auto& pair : structData){
        const auto& data = pair.second;
        if (data.degree < minDegree){
            minDegree = data.degree;
        }
    }    
    return minDegree;
}

int Graph::getMaxDegree(){
    int maxDegree = std::numeric_limits<int>::min();
    auto structData = this->structure->getDataArray();

    for (const auto& pair : structData){
        const auto& data = pair.second;
        if (data.degree > maxDegree){
            maxDegree = data.degree;
        }
    }    
    return maxDegree;
}
void Graph::printSubGraphsToFile(const ReturnSubGraphHeap &subGraphHeap, const std::string &filename) {
    std::string filePath = OUTPUT_PATH + filename;
    std::ofstream outputFile(filePath);  // Abrir o arquivo

    if (!outputFile.is_open()) {  // Verificar se o arquivo foi aberto corretamente
        std::cerr << "Erro ao abrir o arquivo!" << std::endl;
        return;
    }

    ReturnSubGraphHeap heapCopy = subGraphHeap;  // Criar uma cópia do heap, pois priority_queue é destrutiva

    // Percorrer o heap e imprimir cada subgrafo
    while (!heapCopy.empty()) {
        auto currentSubGraph = heapCopy.top();  // Obter o subgrafo no topo (maior)
        heapCopy.pop();  // Remover o subgrafo do heap

        // Escrever o tamanho do subgrafo
        outputFile << "Subgrafo de tamanho: " << currentSubGraph.first << "\n";
        outputFile << "Vértices: ";

        // Escrever os vértices do subgrafo
        for (int vertex : currentSubGraph.second) {
            outputFile << vertex << " ";
        }

        outputFile << "\n\n";  // Nova linha após cada subgrafo
    }

    outputFile.close();  // Fechar o arquivo
}


/// @note Ready for tests
Graph::ReturnGraphDataMap Graph::getBFSTree(int U) {
    std::unordered_map<int, Graph::Data> bfsTree;
    std::unordered_map<int, int> mark(this->vertexAmount);
    std::queue<int> S;
    for (int I = 1; I <= this->vertexAmount; I++){
        mark[I] = 0;
    }
    bfsTree[U].parent = -1;
    bfsTree[U].level = 0;
    mark[U]= 1;
    S.push(U);
    
    while (!S.empty()){
        int V = S.front();
        S.pop();
        for (const int& W : this->structure->getAdjArray(V)){
            if (mark[W] == 0){
                mark[W] = 1;
                bfsTree[W].level = bfsTree[V].level + 1;
                bfsTree[W].parent = V;
                S.push(W);
            }
        }
    }
    return bfsTree;
}
/// @note Ready for tests
Graph::ReturnGraphDataMap Graph::getDFSTree(int U) {
    std::unordered_map<int, Graph::Data> dfsTree;
    std::stack<int> P;
    std::unordered_map<int, int> mark(this->vertexAmount);
    for (int i = 1; i <= vertexAmount; i++){
        mark[i] = 0;
    }
    dfsTree[U].level = 0;
    dfsTree[U].parent = -1;
    P.push(U);
    while (!P.empty()){
        int V = P.top();
        if (mark[V] == 0){
            mark[V] = 1;
            for (const int& W: this->structure->getAdjArray(V)){
                if (mark[W] == 0){
                    dfsTree[W].level = dfsTree[V].level + 1;
                    dfsTree[W].parent = V;
                }
                P.push(W);
            }

        }
    }

    return dfsTree;
}

/// @brief Calculate the Minimal distance between 2 vertices
/// @param U is the first vertice
/// @param V is the second vertice
/// @return the distance between U and V or nullopt case there is no path between them
/// @note Ready for tests
std::optional<int> Graph::getUVDistance(int U, int V) {
    std::unordered_map<int, Data> bfsTreeU = this->getBFSTree(U);
    try{
        return bfsTreeU.at(V).level;
    }
    catch (const std::out_of_range& e){
        return std::nullopt;
    }   
}

int Graph::getExactDiameter(){
    int maxDiameter = 0;
    //int markReference = 0;
    std::unordered_map<int, int> mark;
    for (int V = 1; V <= vertexAmount; V++){
        mark[V] = 0;
    }
    for (int U = 1; U <= vertexAmount; U++){
        int currentDiameter = helper_Diameter(U, mark, U);
        if (currentDiameter > maxDiameter){
            maxDiameter = currentDiameter;
        }
    }
    return maxDiameter;
}

Graph::ReturnSubGraphHeap Graph::getGraphSubComp() {
    Graph::ReturnSubGraphHeap maxHeap;
    auto mark = initVertexMap(0);
    //std::unordered_map<int, int> level;
    for (int U = 1; U <= vertexAmount; U++){
        if (mark[U] == 0){
            std::vector<int> subGraph;
            std::queue<int> S;
            mark[U] = 1;
            S.push(U);
            subGraph.push_back(U);

            while (!S.empty()){
                int V = S.front();
                S.pop();
                for (const int& W : this->structure->getAdjArray(V)){
                    if (mark[W] == 0){
                        mark[W] = 1;
                        S.push(W);
                        subGraph.push_back(W);
                    }
                }
            }
            maxHeap.push(std::make_pair(subGraph.size(), subGraph));
        }
    }
    return maxHeap;

}

int Graph::getApproxDiameter(){
    int maxDiameter = 0;
    std::unordered_map<int, int> mark;
    for (int V = 1; V <= vertexAmount; V++){
        mark[V] = 0;
    }
    for (int U = 1; U <= vertexAmount; U++){
        if (mark[U] == 0){
            int currentDiameter = helper_Diameter(U, mark);
            if (currentDiameter > maxDiameter){
                maxDiameter = currentDiameter;
            }
        }
    }


}


int Graph::helper_Diameter(int U, std::unordered_map<int, int>& mark, int markReference){
    int currentDiameter = 0;
    std::unordered_map<int, int> level;
    std::queue<int> S;
    level[U] = 0;
    mark[U] = U;
    S.push(U);
    while (!S.empty()){
        int V = S.front();
        S.pop();
        for (const int& W : this->structure->getAdjArray(V)){
            if (mark[W] != U){
                mark[W] = U;
                level[W] = level[V] + 1;
                if (level[W] > currentDiameter){
                    currentDiameter = level[W];
                }
                S.push(W);
            }
        }
    }

}


void Graph::printSearchTreeToFile(const ReturnGraphDataMap& searchTree, const std::string& filename, int root) {
    std::string filePath = OUTPUT_PATH + filename;
    std::ofstream outputFile(filePath);  // Abrir o arquivo para escrita

    if (!outputFile.is_open()) {  // Verificar se o arquivo foi aberto corretamente
        std::cerr << "Erro ao abrir o arquivo!" << std::endl;
        return;
    }

    // Escrever o cabeçalho da tabela
    outputFile << "Vértice | Pai    | Nível\n";
    outputFile << "--------|--------|-------\n";

    // Percorrer todos os vértices no mapa de dados da busca
    for (const auto& [vertex, data] : searchTree) {
        // Se o vértice é a raiz (U), o pai deve ser representado por "-"
        std::string parent = (vertex == root) ? "-" : std::to_string(data.parent);

        // Escrever a linha correspondente ao vértice, pai e nível
        outputFile << vertex << "       | " << parent << "      | " << data.level << "\n";
    }

    outputFile.close();  // Fechar o arquivo após a escrita
}


void Graph::printGraphStats(const Graph& graph, bool toFile, const std::string& filename = "") {
    std::ostream* outStream;  // Ponteiro para stream de saída (arquivo ou terminal)
    std::ofstream fileStream;
    std::string filePath = OUTPUT_PATH + filename;
    if (toFile) {
        // Se 'toFile' é verdadeiro, abrir o arquivo
        fileStream.open(filePath);
        if (!fileStream.is_open()) {
            std::cerr << "Erro ao abrir o arquivo!" << std::endl;
            return;
        }
        outStream = &fileStream;  // Usar o arquivo como stream de saída
    } else {
        // Caso contrário, usar std::cout (terminal)
        outStream = &std::cout;
    }

    // Escrever as informações estatísticas
    *outStream << "Número de vértices: " << this->vertexAmount << "\n";
    *outStream << "Número de arestas: " << this->edgeAmount << "\n";
    *outStream << "Grau mínimo: " << this->getMinDegree() << "\n";
    *outStream << "Grau máximo: " << this->getMaxDegree() << "\n";
    *outStream << "Grau médio: " << this->getMean() << "\n";
    *outStream << "Mediana do grau: " << this->getMedian() << "\n";

    // Fechar o arquivo, se foi utilizado
    if (toFile) {
        fileStream.close();
    }
}