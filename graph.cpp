#include <iostream>
#include <vector>
#include <unordered_map>
#include <optional>
#include <algorithm>
#include <unordered_set>
#include <queue>

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

	Graph(const Graph& other)
		: adjacencyList(other.adjacencyList),
		  graphType(other.graphType),
		  weightMode(other.weightMode)
	{}

	Graph& operator=(const Graph& other) {
		if (this != &other) {
			Graph temp(other);
			std::swap(graphType, temp.graphType);
			std::swap(weightMode, temp.weightMode);
			std::swap(adjacencyList, temp.adjacencyList);
		}
		return *this;
	}

	~Graph() = default;

	void addNode(const T& node) {
		if (adjacencyList.find(node) == adjacencyList.end()) {
			adjacencyList[node] = std::vector<Neighbor>();
		}
	}

	size_t numNodes() const {
		return adjacencyList.size();
	}

	bool isEmpty() const {
		return adjacencyList.empty();
	}

	bool containsNode(const T& node) const {
		return adjacencyList.count(node) > 0;
	}

	std::vector<T> getNodes() const {
		std::vector<T> nodes;
		for (const auto& pair : adjacencyList) {
			nodes.push_back(pair.first);
		}
		return nodes;
	}

	void clear() {
		adjacencyList.clear();
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
		if (hasEdge(from, to) || (graphType == GraphType::UNDIRECTED && hasEdge(to, from))) {
			throw std::logic_error("Edge already exists (directly or symmetrically)");
		}
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

	std::optional<WeightType> getEdgeWeight(const T& from, const T& to) const {
		if (adjacencyList.count(from) == 0) {
			return std::nullopt;
		}

		for (const auto& neighborPair : adjacencyList.at(from)) {
			if (neighborPair.first == to) {
				return neighborPair.second;
			}
		}
		return std::nullopt;
	}

	bool setEdgeWeight(const T& from, const T& to, std::optional<WeightType> newWeight) {
		bool updated = false;
		if (adjacencyList.count(from)) {
			for (auto& neighborPair : adjacencyList.at(from)) {
				if (neighborPair.first == to) {
					if (weightMode == WeightMode::WEIGHTED && !newWeight.has_value()) {
						throw std::invalid_argument("Attempting to set weight to nullopt in a weighted graph.");
					}
					else if (weightMode == WeightMode::UNWEIGHTED && newWeight.has_value()) {
						throw std::invalid_argument("Attempting to set a weight in an unweighted graph.");
					}
					neighborPair.second = newWeight;
					updated = true;
					break;
				}
			}
		}
		if (graphType == GraphType::UNDIRECTED && from != to) {
			if (adjacencyList.count(to)) {
				for (auto& neighborPair : adjacencyList.at(to)) {
					if (neighborPair.first == from) {
						neighborPair.second = newWeight;
						updated = true;
						break;
					}
				}
			}
		}
		return updated;
	}

	void removeNode(const T& node) {
		// 1. Supprimer toutes les arêtes entrantes vers ce nœud
		// On doit parcourir tous les autres nœuds pour voir s'ils ont une arête vers 'node'
		for (auto& pair : adjacencyList) {
			//T currentNode = pair.first;
			std::vector<Neighbor>& neighbors = pair.second;
			// Suppression des éléments
			neighbors.erase(std::remove_if(neighbors.begin(), neighbors.end(),
			[&](const Neighbor& neighbor) {
				return neighbor.first == node;
			}),
			neighbors.end());
		}
		// 2. Supprimer le nœud lui-même de la liste d'adjacence
		adjacencyList.erase(node);
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

	bool hasPath(const T& startNode, const T& endNode) const {
		// startNode and endNode should exist
		if (adjacencyList.count(startNode) == 0 || adjacencyList.count(endNode) == 0) {
			return false;
		}

		if (startNode == endNode) {
			return true;
		}

		std::queue<T> q;
		std::unordered_set<T> visited;

		q.push(startNode);
		visited.insert(startNode);

		while (!q.empty()) {
			T currentNode = q.front();
			q.pop();

			for (const auto& neighborPair : getNeighbors(currentNode)) {
				T neighbor = neighborPair.first;
				if (neighbor == endNode) {
					return true;
				}
				if (visited.find(neighbor) == visited.end()) { // Or !visited.count(neighbor)
					visited.insert(neighbor);
					q.push(neighbor);
				}
			}
		}
		return false;
	}

	bool checkUndirectedConsistency() const {
		if (graphType != GraphType::UNDIRECTED) return true;
		for (const auto& [from, neighbors] : adjacencyList) {
			for (const auto& [to, _] : neighbors) {
				auto it = adjacencyList.find(to);
				if (it == adjacencyList.end()) return false;
				bool found = false;
				for (const auto& [back, __] : it->second) {
					if (back == from) {
						found = true;
						break;
					}
				}
				if (!found) return false;
			}
		}
		return true;
	}

	class NodeIterator {
	public:
		using iterator_category = std::forward_iterator_tag;
		using value_type = T;
		using difference_type = std::ptrdiff_t;
		using pointer = const T*;
		using reference = const T&;

		NodeIterator(typename std::unordered_map<T, std::vector<Neighbor>>::const_iterator it)
			: current_it(it) {}

		const T& operator*() const {
			return current_it->first; // Retourne la clé (le nœud) de la paire de la map
		}

		NodeIterator& operator++() {
			++current_it;
			return *this;
		}
		NodeIterator operator++(int) {
			NodeIterator temp = *this;
			++(*this);
			return temp;
		}

		bool operator==(const NodeIterator& other) const {
			return current_it == other.current_it;
		}

		bool operator!=(const NodeIterator& other) const {
			return current_it != other.current_it;
		}

	private:
		typename std::unordered_map<T, std::vector<Neighbor>>::const_iterator current_it;
	};

	NodeIterator begin() const {
		return NodeIterator(adjacencyList.begin());
	}

	NodeIterator end() const {
		return NodeIterator(adjacencyList.end());
	}

	class EdgeIterator {
	public:
		using iterator_category = std::forward_iterator_tag;
		using value_type = Neighbor;
		using difference_type = std::ptrdiff_t;
		using pointer = const Neighbor*;
		using reference = const Neighbor&;

		EdgeIterator(typename std::vector<Neighbor>::const_iterator it)
			: current(it) {}

		reference operator*() const {
			return *current;
		}
		pointer operator->() const {
			return &(*current);
		}

		EdgeIterator& operator++() {
			++current;
			return *this;
		}
		EdgeIterator operator++(int) {
			EdgeIterator tmp = *this;
			++(*this);
			return tmp;
		}

		bool operator==(const EdgeIterator& other) const {
			return current == other.current;
		}
		bool operator!=(const EdgeIterator& other) const {
			return current != other.current;
		}

	private:
		typename std::vector<Neighbor>::const_iterator current;
	};

	EdgeIterator edgeBegin(const T& node) const {
		auto it = adjacencyList.find(node);
		if (it != adjacencyList.end()) {
			return EdgeIterator(it->second.begin());
		}
		return edgeEnd(node);
	}

	EdgeIterator edgeEnd(const T& node) const {
		auto it = adjacencyList.find(node);
		if (it != adjacencyList.end()) {
			return EdgeIterator(it->second.end());
		}
		return EdgeIterator({}); // empty iterator
	}

	class EdgeRange {
	public:
		EdgeRange(EdgeIterator begin, EdgeIterator end)
			: b(begin), e(end) {}
		EdgeIterator begin() const {
			return b;
		}
		EdgeIterator end() const {
			return e;
		}
	private:
		EdgeIterator b, e;
	};

	EdgeRange edges(const T& node) const {
		return EdgeRange(edgeBegin(node), edgeEnd(node));
	}

private:
	std::unordered_map<T, std::vector<Neighbor>> adjacencyList;
	GraphType graphType;
	WeightMode weightMode;
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
	std::cout << "Undirected Graph:\n";
	undirectedGraph.removeNode("D");
	undirectedGraph.printGraph();
	std::cout << undirectedGraph.hasPath("A","E") << std::endl;

	Graph<int> originalGraph(GraphType::DIRECTED, WeightMode::UNWEIGHTED);
	originalGraph.addEdge(1, 2);
	originalGraph.addEdge(2, 3);
	originalGraph.addNode(4);

	std::cout << "Original Graph:\n";
	originalGraph.printGraph();

	// Création d'une copie
	Graph<int> copiedGraph = originalGraph; // Appel du constructeur de copie
	// Ou : Graph<int> copiedGraph(originalGraph);

	std::cout << "\nCopied Graph (initial):\n";
	copiedGraph.printGraph();

	// Modifier le graphe original
	originalGraph.addEdge(3, 4);
	originalGraph.removeNode(1);

	std::cout << "\nOriginal Graph (after modifications):\n";
	originalGraph.printGraph();

	std::cout << "\nCopied Graph (should be unchanged):\n";
	copiedGraph.printGraph(); // Le graphe copié ne devrait pas être affecté

	// Test de l'opérateur d'affectation
	Graph<int> anotherGraph;
	anotherGraph.addEdge(10, 11);
	std::cout << "\nAnother Graph (initial):\n";
	anotherGraph.printGraph();

	anotherGraph = originalGraph; // Appel de l'opérateur d'affectation par copie
	std::cout << "\nAnother Graph (after assignment from original):\n";
	anotherGraph.printGraph();

	std::vector<std::string> allStringNodes = undirectedGraph.getNodes();
	std::cout << "Noeuds du graphe non dirige: ";
	for (const std::string& node : allStringNodes) {
		std::cout << node << " ";
	}
	std::cout << std::endl;

	Graph<int> myGraph(GraphType::DIRECTED);
	myGraph.addNode(10);
	myGraph.addNode(20);
	myGraph.addNode(30);
	myGraph.addEdge(10, 20);
	myGraph.addEdge(20, 30);

	std::cout << "Parcours des noeuds avec un NodeIterator (boucle for-range):\n";
	for (const int& node : myGraph) { // <-- C'est ça la magie !
		std::cout << "  Noeud: " << node << std::endl;
	}

	std::cout << "\nParcours des noeuds avec un NodeIterator (boucle for classique):\n";
	for (Graph<int>::NodeIterator it = myGraph.begin(); it != myGraph.end(); ++it) {
		std::cout << "  Noeud: " << *it << std::endl;
	}

	// Utilisation avec un algorithme de la STL (exemple simple)
	std::cout << "\nRecherche d'un noeud avec std::find:\n";
	int nodeToFind = 20;
	auto it_found = std::find(myGraph.begin(), myGraph.end(), nodeToFind);

	if (it_found != myGraph.end()) {
		std::cout << "  Noeud " << nodeToFind << " trouvé dans le graphe.\n";
	}
	else {
		std::cout << "  Noeud " << nodeToFind << " non trouvé dans le graphe.\n";
	}

	// Tester un nœud qui n'existe pas
	nodeToFind = 40;
	it_found = std::find(myGraph.begin(), myGraph.end(), nodeToFind);
	if (it_found != myGraph.end()) {
		std::cout << "  Noeud " << nodeToFind << " trouvé dans le graphe.\n";
	}
	else {
		std::cout << "  Noeud " << nodeToFind << " non trouvé dans le graphe.\n";
	}

	Graph<std::string> g(GraphType::DIRECTED, WeightMode::WEIGHTED);
	g.addEdge("A", "B", 3.0);
	g.addEdge("A", "C", 5.0);
	g.addEdge("A", "D", 2.0);

	for (auto it = g.edgeBegin("A"); it != g.edgeEnd("A"); ++it) {
		auto [to, weight] = *it;
		std::cout << "A -> " << to;
		if (weight) std::cout << " (" << *weight << ")";
		std::cout << '\n';
	}

	for (const auto& [to, weight] : g.edges("A")) {
		std::cout << "A -> " << to << '\n';
	}

	return 0;
}
