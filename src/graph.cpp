#include "graph.hpp"
#include <unordered_map>
#include <bitset>
#include <limits>
#include <stdexcept>

std::pair<std::string, int> extractNumberName(const std::string& numberName){
    size_t pos = numberName.find(',');
    int number;
    std::string name;
    if (pos == std::string::npos){
        throw std::invalid_argument("string format is invalid");
    }
    number = std::stoi(numberName.substr(0, pos));
    name = numberName.substr(pos + 1);
    
    
    return std::make_pair(name, number);
}



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
    this->named = false;

}

Graph::Graph(std::string &path, GraphStructure structure, bool isDirected, bool isWeighted, std::string &names) {
    Graph(path, structure, isDirected, isWeighted);
    this->named = true;

    std::string line;
    std::ifstream labelsFile(names);

    if (!labelsFile.is_open()){
        throw std::runtime_error("Error! Couldn't open label file");
    }

    while (getline(labelsFile, line)){
        auto name = extractNumberName(line);
        this->labelToVertexMap[name.first] = name.second;
        this->vertexToLabelMap[name.second] = name.first;
    }

}

void Graph::helper_init(std::string &path, GraphStructure structure, bool isDirected, bool isWeighted) {
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
        P.pop();
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

std::unordered_map<int, Graph::DijkstraNode> Graph::getDijkstraTree(int U, bool useHeap) {
    if (!hasWeight) {
        throw std::logic_error("Error! Cannot use dijkstra in weightless graph");
    }
    else if (!useHeap) {
        return this->getDijkstraTreeDefault(U);
    }

    // Priority queue for Dijkstra (min-heap), stores pairs of (distance, vertex)
    using QueueElement = std::pair<double, int>;
    static std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<QueueElement>> pq;
    static std::vector<double> dist(getVertexAmount() + 1, std::numeric_limits<double>::max());
    static std::unordered_map<int, DijkstraNode> dijkstraTree;

    // Clear any data left from previous runs
    dist.assign(getVertexAmount() + 1, std::numeric_limits<double>::max());
    dijkstraTree.clear();
    while (!pq.empty()) pq.pop();

    // Initialization of vertices and distances
    dist[U] = 0;
    dijkstraTree[U].distance = 0;
    dijkstraTree[U].parent = -1;
    pq.push({0, U});

    while (!pq.empty()) {
        double currentDist = pq.top().first;
        int V = pq.top().second;
        pq.pop();

        // Ignore outdated entries in the priority queue
        if (currentDist > dist[V]) continue;

        for (auto& neighbor : this->structure->getAdjWeightedArray(V)) {
            int W = neighbor.first;
            double weight = neighbor.second;

            if (weight < 0) {
                throw std::logic_error("Support for negative weights is not yet available.");
            }

            // Update the shortest distance if found
            double newDist = dist[V] + weight;
            if (dist[W] > newDist) {
                dist[W] = newDist;
                dijkstraTree[W].distance = newDist;
                dijkstraTree[W].parent = V;
                pq.push({newDist, W});
            }
        }
    }

    return std::move(dijkstraTree);
}

std::unordered_map<int, Graph::DijkstraNode> Graph::getDijkstraTreeDefault(int U) {
    if (!hasWeight) {
        throw std::logic_error("Error! Cannot use Dijkstra in a weightless graph");
    }

    double inf = std::numeric_limits<double>::max();
    std::unordered_map<int, DijkstraNode> dijkstraTree;
    std::bitset<10000000> S;  // Bitset for tracking processed nodes (up to 10 million nodes)

    // Initialize the starting node distance
    dijkstraTree[U].distance = 0;
    dijkstraTree[U].parent = -1;

    while (S.count() < this->getVertexAmount()) {
        int V = -1;
        double minDist = inf;

        // Select the unprocessed node with the minimum distance
        for (const auto& [node, data] : dijkstraTree) {
            if (!S[node] && data.distance < minDist) {
                V = node;
                minDist = data.distance;
            }
        }

        // If no minimum node was found, the remaining vertices are unreachable
        if (V == -1) break;

        S.set(V);  // Mark this node as processed

        // Relaxation step for all adjacent vertices of V
        for (const auto& neighbor : this->structure->getAdjWeightedArray(V)) {
            int W = neighbor.first;
            double weight = neighbor.second;

            if (weight < 0) {
                throw std::logic_error("Support for negative weights is not yet available.");
            }

            // Relax the edge if a shorter path is found
            double newDist = dijkstraTree[V].distance + weight;
            if (!dijkstraTree.count(W) || dijkstraTree[W].distance > newDist) {
                dijkstraTree[W].distance = newDist;
                dijkstraTree[W].parent = V;
            }
        }
    }

    return dijkstraTree;
}

/// @brief Calculate the Minimal distance between 2 vertices
/// @param U is the first vertice
/// @param V is the second vertice
/// @return the distance between U and V or nullopt case there is no path between them
/// @note Ready for tests
int Graph::getUVDistance(int U, int V) {
    std::unordered_map<int, Data> bfsTreeU = this->getBFSTree(U);
    try{
        return bfsTreeU.at(V).level;
    }
    catch (const std::out_of_range& e){
        return std::numeric_limits<int>::max();
    }   
}

double Graph::getUVDistance(int U, int V, bool useHeap) {
    if (!hasWeight){
        throw std::logic_error("Cant measure weight distance in weightless graph");
    }
    int dist;
    try {
        if (useHeap) {
            return getDijkstraTree(U, useHeap).at(V).distance;
        }
        
        else{
            return getDijkstraTreeDefault(U).at(V).distance;
        }
    }
    catch (std::out_of_range &e){
        return std::numeric_limits<double>::max();
    }
}

double Graph::getDijkstraMinDistance(int U, int V, bool useHeap) {
    if (!hasWeight){
        throw std::logic_error("Cant measure weight distance in weightless graph");
    }

    auto dijkstraTree = getDijkstraTree(U, useHeap);
    return getDijkstraMinDistance(dijkstraTree, V);
    
}

double Graph::getDijkstraMinDistance(std::unordered_map<int, DijkstraNode> &dijkstraTree, int V) {
    if (!hasWeight){
        throw std::logic_error("Cant measure weight distance in weightless graph");
    }

    if (dijkstraTree.find(V) != dijkstraTree.end()) {
        return dijkstraTree[V].distance;
    }
    return std::numeric_limits<double>::max();
}

std::stack<int> Graph::getPathUV(int U, int V, bool useHeap) {
    std::stack<int> path;
    int parent;
    if (!hasWeight){
        throw std::logic_error("Weightless Graph doesn't have weighted Path");
    }
    auto dijkstraTree = useHeap ? this->getDijkstraTree(U, useHeap) : this->getDijkstraTreeDefault(U);
    if (dijkstraTree.find(V) != dijkstraTree.end()){
        parent = dijkstraTree[V].parent;
        path.push(V);
        while (parent != U){
            path.push(parent);
            parent = dijkstraTree[parent].parent;
        }
        path.push(U);
    }
    return path;
    
}

std::queue<std::string> Graph::getNamedPathUV(std::string name1, std::string name2) {
    int vertex1 = getVertexByLabel(name1);
    int vertex2 = getVertexByLabel(name2);
    auto path = this->getPathUV(vertex1, vertex2, true);
    std::queue<std::string> nameQueue;
    while (!path.empty()) {
        auto number = path.top();
        path.pop();
        nameQueue.push(this->getLabelByVertex(number));
    }
    return nameQueue;

}

int Graph::getVertexByLabel(std::string name) {
    if (!this->named) {
        throw std::logic_error("Error! Graph has no names");
    }
    if (this->labelToVertexMap.find(name) == labelToVertexMap.end()) {
        throw std::out_of_range("Name not in graph");
    }
    return labelToVertexMap[name];
}

std::string Graph::getLabelByVertex(int vertex) {
    if (!this->named) {
        throw std::logic_error("Error! Graph has no names");
    }
    if (this->vertexToLabelMap.find(vertex) == vertexToLabelMap.end()) {
        throw std::out_of_range("Vertex not in graph");
    }
    return vertexToLabelMap[vertex];
}

std::deque<int> Graph::getDijkstraMinPathVertices(int U, int V, bool useHeap) {
    if (!hasWeight){
        throw std::logic_error("Weightless Graph doesn't have weighted Path");
    }
    auto dijkstraTree = useHeap ? this->getDijkstraTree(U, useHeap) : this->getDijkstraTreeDefault(U);
    return this->getDijkstraMinPathVertices(U, V, dijkstraTree);
}

std::deque<int> Graph::getDijkstraMinPathVertices(int U, int V, std::unordered_map<int, DijkstraNode> &dijkstraTree) {
    if (!hasWeight){
        throw std::logic_error("Weightless Graph doesn't have weighted Path");
    }
    std::deque<int> path;
    int parent;
    if (dijkstraTree.find(V) != dijkstraTree.end()){
        parent = dijkstraTree[V].parent;
        path.push_back(V);
        while (parent != U){
            path.push_back(parent);
            parent = dijkstraTree[parent].parent;
        }
        path.push_back(U);
    }
    return path;
}

std::deque<std::string> Graph::getDijkstraMinPathLabels(std::string labelU, std::string labelV, bool useHeap) {
    if (!hasWeight){
        throw std::logic_error("Weightless Graph doesn't have weighted Path");
    }
    int U = getVertexByLabel(labelU);
    int V = getVertexByLabel(labelV);
    auto dijkstraTree = useHeap ? this->getDijkstraTree(U, useHeap) : this->getDijkstraTreeDefault(U);
    return this->getDijkstraMinPathLabels(labelU, labelV, dijkstraTree);

}

std::deque<std::string> Graph::getDijkstraMinPathLabels(std::string labelU, std::string labelV, std::unordered_map<int, DijkstraNode> &dijkstraTree) {
    if (!hasWeight){
        throw std::logic_error("Weightless Graph doesn't have weighted Path");
    }
    int U = getVertexByLabel(labelU);
    int V = getVertexByLabel(labelV);
    std::deque<std::string> path;
    int parent;
    if (dijkstraTree.find(V) != dijkstraTree.end()){
        parent = dijkstraTree[V].parent;
        
        path.push_back(this->getLabelByVertex(V));
        while (parent != U){
            path.push_back(this->getLabelByVertex(parent));
            parent = dijkstraTree[parent].parent;
        }
        path.push_back(this->getLabelByVertex(U));
    }
    return path;
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

int Graph::getVertexAmount() {
    return this->vertexAmount;
}

int Graph::getEdgeAmount() {
    return this->edgeAmount;
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

    return maxDiameter;
}



int Graph::helper_Diameter(int U, std::unordered_map<int, int> &mark, int markReference)
{
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

    return currentDiameter;
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