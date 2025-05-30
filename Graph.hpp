#pragma once
#include <unordered_map>
#include <vector>
#include <optional>
#include <stdexcept>
#include <sstream>
#include <iostream>
#include <queue>
#include <unordered_set>
#include <limits>

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

	Graph(GraphType type = GraphType::DIRECTED, WeightMode wMode = WeightMode::UNWEIGHTED);
	Graph(const Graph& other);
	Graph& operator=(const Graph& other);
	~Graph();

	// === Nodes ===
	void addNode(const T& node);
	void removeNode(const T& node);
	std::vector<T> getNodes() const;
	size_t numNodes() const;
	bool containsNode(const T& node) const;

	// === Edges ===
	void addEdge(const T& from, const T& to, std::optional<WeightType> weight = std::nullopt);
	void removeEdge(const T& from, const T& to);
	bool hasEdge(const T& from, const T& to) const;
	std::optional<WeightType> getEdgeWeight(const T& from, const T& to) const;
	bool setEdgeWeight(const T& from, const T& to, std::optional<WeightType> newWeight);
	size_t numEdges() const;

	// === Utility ===
	bool isEmpty() const;
	bool hasPath(const T& startNode, const T& endNode) const;
	std::vector<Neighbor> getNeighbors(const T& node) const;
	bool checkUndirectedConsistency() const;
	void clear();

	std::vector<T> shortestPath(const T& start, const T& end) const;

	// === Export ===
	void printGraph() const;
	std::string toDot(const std::string& graphName = "G") const;

	// === Iterators ===
	class NodeIterator;
	NodeIterator begin() const;
	NodeIterator end() const;

	class EdgeIterator;
	EdgeIterator edgeBegin(const T& node) const;
	EdgeIterator edgeEnd(const T& node) const;

	class EdgeRange;
	EdgeRange edges(const T& node) const;

private:
	std::unordered_map<T, std::vector<Neighbor>> adjacencyList;
	GraphType graphType;
	WeightMode weightMode;
};

// constructors

template <typename T, typename WeightType>
Graph<T, WeightType>::Graph(GraphType type, WeightMode wMode)
	: graphType(type), weightMode(wMode)
{}

template <typename T, typename WeightType>
Graph<T, WeightType>::Graph(const Graph& other)
	: adjacencyList(other.adjacencyList),
	  graphType(other.graphType),
	  weightMode(other.weightMode)
{}

template <typename T, typename WeightType>
Graph<T, WeightType>& Graph<T, WeightType>::operator=(const Graph& other)
{
	if (this != &other) {
		Graph temp(other);
		std::swap(graphType, temp.graphType);
		std::swap(weightMode, temp.weightMode);
		std::swap(adjacencyList, temp.adjacencyList);
	}
	return *this;
}

template <typename T, typename WeightType>
Graph<T, WeightType>::~Graph() = default;

// methods

template <typename T, typename WeightType>
bool Graph<T, WeightType>::isEmpty() const
{
	return adjacencyList.empty();
}

template <typename T, typename WeightType>
void Graph<T, WeightType>::clear()
{
	adjacencyList.clear();
}

template <typename T, typename WeightType>
std::vector<typename Graph<T, WeightType>::Neighbor> Graph<T, WeightType>::getNeighbors(const T& node) const
{
	if (adjacencyList.count(node)) {
		return adjacencyList.at(node);
	}
	return {};
}

template <typename T, typename WeightType>
bool Graph<T, WeightType>::hasPath(const T& startNode, const T& endNode) const
{
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

template <typename T, typename WeightType>
bool Graph<T, WeightType>::checkUndirectedConsistency() const
{
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

template <typename T, typename WeightType>
std::string Graph<T, WeightType>::toDot(const std::string& graphName) const
{
	std::ostringstream oss;
	bool directed = graphType == GraphType::DIRECTED;

	oss << (directed ? "digraph " : "graph ") << graphName << " {\n";

	std::unordered_set<std::string> printedEdges;

	for (const auto& [from, neighbors] : adjacencyList) {
		for (const auto& [to, weight] : neighbors) {
			// Pour éviter les doublons dans les graphes non orientés
			if (!directed && from > to) continue;

			oss << "  \"" << from << "\" " << (directed ? "->" : "--") << " \"" << to << "\"";

			if (weightMode == WeightMode::WEIGHTED && weight.has_value()) {
				oss << " [label=\"" << *weight << "\"]";
			}
			oss << ";\n";
		}
	}
	oss << "}\n";
	return oss.str();
}

template <typename T, typename WeightType>
void Graph<T, WeightType>::printGraph() const
{
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


template <typename T, typename WeightType>
std::vector<T> Graph<T, WeightType>::shortestPath(const T& start, const T& end) const
{
	if (!containsNode(start) || !containsNode(end)) {
		return {};  // No path if start or end not present
	}

	std::unordered_map<T, WeightType> distances;
	std::unordered_map<T, T> predecessors;
	std::unordered_set<T> visited;

	auto compare = [&](const T& lhs, const T& rhs) {
		return distances[lhs] > distances[rhs];
	};

	std::priority_queue<T, std::vector<T>, decltype(compare)> queue(compare);

	for (const auto& [node, _] : adjacencyList) {
		distances[node] = std::numeric_limits<WeightType>::max();
	}

	distances[start] = 0;
	queue.push(start);

	while (!queue.empty()) {
		T current = queue.top();
		queue.pop();

		if (visited.count(current)) continue;
		visited.insert(current);

		for (const auto& [neighbor, weightOpt] : adjacencyList.at(current)) {
			WeightType edgeWeight = weightOpt.value_or(1);
			WeightType newDist = distances[current] + edgeWeight;

			if (newDist < distances[neighbor]) {
				distances[neighbor] = newDist;
				predecessors[neighbor] = current;
				queue.push(neighbor);
			}
		}
	}

	if (!predecessors.count(end)) {
		return {};  // No path found
	}

	// Reconstruct path
	std::vector<T> path;
	for (T at = end; at != start; at = predecessors[at]) {
		path.push_back(at);
	}
	path.push_back(start);
	std::reverse(path.begin(), path.end());
	return path;
}

// Nodes

template <typename T, typename WeightType>
void Graph<T, WeightType>::addNode(const T& node)
{
	if (adjacencyList.find(node) == adjacencyList.end()) {
		adjacencyList[node] = std::vector<Neighbor>();
	}
}

template <typename T, typename WeightType>
std::vector<T> Graph<T, WeightType>::getNodes() const
{
	std::vector<T> nodes;
	for (const auto& pair : adjacencyList) {
		nodes.push_back(pair.first);
	}
	return nodes;
}

template <typename T, typename WeightType>
void Graph<T, WeightType>::removeNode(const T& node)
{
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

template <typename T, typename WeightType>
size_t Graph<T, WeightType>::numNodes() const
{
	return adjacencyList.size();
}

template <typename T, typename WeightType>
bool Graph<T, WeightType>::containsNode(const T& node) const
{
	return adjacencyList.count(node) > 0;
}

// Edges

template <typename T, typename WeightType>
void Graph<T, WeightType>::addEdge(const T& from, const T& to, std::optional<WeightType> weight)
{
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

template <typename T, typename WeightType>
void Graph<T, WeightType>::removeEdge(const T& from, const T& to)
{
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

template <typename T, typename WeightType>
size_t Graph<T, WeightType>::numEdges() const
{
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

template <typename T, typename WeightType>
bool Graph<T, WeightType>::hasEdge(const T& from, const T& to) const
{
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

template <typename T, typename WeightType>
std::optional<WeightType> Graph<T, WeightType>::getEdgeWeight(const T& from, const T& to) const
{
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

template <typename T, typename WeightType>
bool Graph<T, WeightType>::setEdgeWeight(const T& from, const T& to, std::optional<WeightType> newWeight)
{
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

// Iterators

// NodeIterator

template <typename T, typename WeightType>
class Graph<T, WeightType>::NodeIterator {
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

template <typename T, typename WeightType>
class Graph<T, WeightType>::NodeIterator Graph<T, WeightType>::begin() const {
	return NodeIterator(adjacencyList.begin());
}

template <typename T, typename WeightType>
class Graph<T, WeightType>::NodeIterator Graph<T, WeightType>::end() const {
	return NodeIterator(adjacencyList.end());
}

// EdgeIterator

template <typename T, typename WeightType>
class Graph<T, WeightType>::EdgeIterator {
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

template <typename T, typename WeightType>
typename Graph<T, WeightType>::EdgeIterator Graph<T, WeightType>::edgeBegin(const T& node) const
{
	auto it = adjacencyList.find(node);
	if (it != adjacencyList.end()) {
		return EdgeIterator(it->second.begin());
	}
	return edgeEnd(node);
}

template <typename T, typename WeightType>
typename Graph<T, WeightType>::EdgeIterator Graph<T, WeightType>::edgeEnd(const T& node) const
{
	auto it = adjacencyList.find(node);
	if (it != adjacencyList.end()) {
		return EdgeIterator(it->second.end());
	}
	return EdgeIterator({}); // empty iterator
}

// EdgeRange

template <typename T, typename WeightType>
class Graph<T, WeightType>::EdgeRange {
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

template <typename T, typename WeightType>
typename Graph<T, WeightType>::EdgeRange Graph<T, WeightType>::edges(const T& node) const
{
	return EdgeRange(edgeBegin(node), edgeEnd(node));
}