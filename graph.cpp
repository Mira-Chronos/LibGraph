#include <iostream>
#include <vector>
#include <unordered_map>
#include <optional>

enum class GraphType {
    DIRECTED,
    UNDIRECTED
};

enum class WeightMode {
    UNDEFINED,
    WEIGHTED,
    UNWEIGHTED
};

template <typename T, typename WeightType = double>
class Graph {
public:
    using Neighbor = std::pair<T, std::optional<WeightType>>;

Graph(GraphType type = GraphType::DIRECTED, WeightMode wMode = WeightMode::UNWEIGHTED)
    : graphType(type), weightMode(wMode) {}

    void addNode(const T& node) {
        if (adjacencyList.find(node) == adjacencyList.end()) {
            adjacencyList[node] = std::vector<Neighbor>();
        }
    }

    void addEdge(const T& from, const T& to, std::optional<WeightType> weight = std::nullopt) {
        addNode(from);
        addNode(to);

        if ((weightMode == WeightMode::WEIGHTED && !weight.has_value()) ||
                   (weightMode == WeightMode::UNWEIGHTED && weight.has_value())) {
            throw std::invalid_argument("Attempting to add an edge with inconsistent weight mode.");
        }
        adjacencyList[from].push_back({to, weight});
        if (graphType == GraphType::UNDIRECTED && from != to) {
            adjacencyList[to].push_back({from, weight});
        }
    }

    void printGraph() const {
        for (const auto& pair : adjacencyList) {
            std::cout << "Node " << pair.first << ": ";
            for (const auto& neighborPair : pair.second) {
                std::cout << "(" << neighborPair.first;
                if (neighborPair.second.has_value()) {
                    std::cout << ", " << neighborPair.second.value();
                }
                std::cout << ") ";
            }
            std::cout << std::endl;
        }
    }

    std::vector<Neighbor> getNeighbors(const T& node) const {
        if (adjacencyList.count(node)) {
            return adjacencyList.at(node);
        }
        return {};
    }

private:
    std::unordered_map<T, std::vector<Neighbor>> adjacencyList;
    WeightMode weightMode;
    GraphType graphType;
};

int main()
{
    Graph<int> directedGraph(GraphType::DIRECTED);
    directedGraph.addEdge(1, 2);
    directedGraph.addEdge(2, 1);
    directedGraph.addEdge(2, 3);
    directedGraph.addEdge(3, 1);
    std::cout << "Directed Graph:\n";
    directedGraph.printGraph();

    std::cout << "\n";

    Graph<std::string> undirectedGraph(GraphType::UNDIRECTED, WeightMode::WEIGHTED);
    undirectedGraph.addEdge("A", "B", 1.0);
    undirectedGraph.addEdge("B", "C", 2.0);
    undirectedGraph.addEdge("A", "C", 3.0);
    undirectedGraph.addEdge("B", "E", 1.0);
    undirectedGraph.addEdge("C", "E", 1.0);
    undirectedGraph.addEdge("D", "E", 1.0);
    undirectedGraph.addEdge("C", "D", 1.0);
    std::cout << "Undirected Graph:\n";
    undirectedGraph.printGraph();
    std::cout << "Neighbors : " << " E" << std::endl;
    auto tmp = undirectedGraph.getNeighbors("E");
    for (const auto& n : tmp) {
        std::cout << "\t"<< n.first << std::endl;
    }


    return 0;
}