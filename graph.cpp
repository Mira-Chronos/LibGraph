#include <iostream>
#include <vector>
#include <unordered_map>
#include <optional>
#include <algorithm>

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

	size_t numNodes() const {
		return adjacencyList.size(); // La taille de la map est le nombre de nœuds
	}

	bool isEmpty() const {
		return adjacencyList.empty();
	}

	size_t numEdges() const {
		size_t edgeCount = 0;
		for (const auto& pair : adjacencyList) {
			//std::cout << pair.first << " -> " << pair.second.size() << std::endl;
			edgeCount += pair.second.size();
		}
		if (graphType == GraphType::UNDIRECTED) {
			edgeCount /= 2;
		}
		return edgeCount;
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

	bool hasEdge(const T& from, const T& to) const {
		if (adjacencyList.count(from) == 0) {
			return false;
		}
		for (const auto& neighborPair : adjacencyList.at(from)) {
			if (neighborPair.first == to) {
				return true;
			}
		}
		return false;
	}

	void removeEdge(const T& from, const T& to) {
		// remove 'from' -> 'to'
		if (adjacencyList.count(from)) {
			std::vector<Neighbor>& neighbors = adjacencyList.at(from);
			neighbors.erase(std::remove_if(neighbors.begin(), neighbors.end(),
			[&](const Neighbor& neighbor) {
				return neighbor.first == to;
			}),
			neighbors.end());
		}
		// if undirected graph, remove  'to' -> 'from'
		if (graphType == GraphType::UNDIRECTED && from != to) {
			if (adjacencyList.count(to)) {
				std::vector<Neighbor>& neighbors = adjacencyList.at(to);
				neighbors.erase(std::remove_if(neighbors.begin(), neighbors.end(),
				[&](const Neighbor& neighbor) {
					return neighbor.first == from;
				}),
				neighbors.end());
			}
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
	std::cout << directedGraph.numNodes() << std::endl;
	std::cout << directedGraph.numEdges() << std::endl;

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
	undirectedGraph.removeEdge("B", "G");
	std::cout << "Neighbors : " << " E" << std::endl;
	auto tmp = undirectedGraph.getNeighbors("E");
	for (const auto& n : tmp) {
		std::cout << "\t"<< n.first << std::endl;
	}
	std::cout << undirectedGraph.numNodes() << std::endl;
	std::cout << undirectedGraph.numEdges() << std::endl;
	undirectedGraph.printGraph();
	return 0;
}