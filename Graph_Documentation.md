
# `Graph<T, WeightType>` Class Documentation

Represents a directed or undirected graph, weighted or unweighted, with generic nodes.

## Template

```cpp
template <typename T, typename WeightType = double>
class Graph;
```

- `T`: The type used for nodes (must be hashable for `unordered_map`)
- `WeightType`: The type used for edge weights (optional, defaults to `double`)

## Public Types

```cpp
using Neighbor = std::pair<T, std::optional<WeightType>>;
```
- Alias representing a neighbor node and an optional weight.

## Constructors & Destructor

### `Graph(GraphType type = DIRECTED, WeightMode wMode = UNWEIGHTED)`
Constructs an empty graph with the specified type and weight mode.

### `Graph(const Graph& other)`
Copy constructor.

### `Graph& operator=(const Graph& other)`
Copy assignment operator using copy-and-swap idiom.

### `~Graph()`
Destructor (default implementation is sufficient).

## Node Management

### `void addNode(const T& node)`
Adds a node if it doesn't already exist.

### `void removeNode(const T& node)`
Removes the node and all associated edges.

### `std::vector<T> getNodes() const`
Returns a list of all nodes in the graph.

### `size_t numNodes() const`
Returns the number of nodes.

### `bool containsNode(const T& node) const`
Checks if the node exists in the graph.

## Edge Management

### `void addEdge(const T& from, const T& to, std::optional<WeightType> weight = std::nullopt)`
Adds an edge between two nodes (and the reverse if undirected). Validates the weight mode.

### `void removeEdge(const T& from, const T& to)`
Removes an edge (and its reverse if undirected).

### `bool hasEdge(const T& from, const T& to) const`
Checks if an edge exists between two nodes.

### `std::optional<WeightType> getEdgeWeight(const T& from, const T& to) const`
Returns the weight of the edge if it exists.

### `bool setEdgeWeight(const T& from, const T& to, std::optional<WeightType> newWeight)`
Updates the weight of an existing edge.

### `size_t numEdges() const`
Returns the total number of edges (counts each undirected edge only once).

## Utility Functions

### `bool isEmpty() const`
Returns true if the graph has no nodes.

### `bool hasPath(const T& startNode, const T& endNode) const`
Checks whether a path exists between two nodes (BFS).

### `std::vector<Neighbor> getNeighbors(const T& node) const`
Returns the list of neighbors for a given node.

### `bool checkUndirectedConsistency() const`
For undirected graphs, checks if each edge has a symmetric counterpart.

### `void clear()`
Clears the entire graph (nodes and edges).

## Export / Debug

### `void printGraph() const`
Prints the graph’s structure to the standard output.

### `std::string toDot(const std::string& graphName = "G") const`
Generates a DOT format string for use with Graphviz.

## Iterators

### `class NodeIterator`
Iterator for looping over the nodes.

### `NodeIterator begin() const`, `NodeIterator end() const`
Returns iterators for node traversal.

### `class EdgeIterator`
Iterator for looping over a node’s neighbors.

### `EdgeIterator edgeBegin(const T& node) const`
Returns an iterator to the beginning of the node’s neighbors.

### `EdgeIterator edgeEnd(const T& node) const`
Returns an iterator to the end of the node’s neighbors.

### `class EdgeRange`
A utility object to simplify edge iteration with range-based for loops.

### `EdgeRange edges(const T& node) const`
Returns an `EdgeRange` object for iterating over a node’s edges.

## Private Attributes

```cpp
std::unordered_map<T, std::vector<Neighbor>> adjacencyList;
GraphType graphType;
WeightMode weightMode;
```

- `adjacencyList`: Adjacency list representing graph structure.
- `graphType`: Whether the graph is directed or undirected.
- `weightMode`: Whether the graph is weighted or unweighted.

## Example Usage

```cpp
Graph<std::string> g(GraphType::UNDIRECTED, WeightMode::WEIGHTED);
g.addNode("A");
g.addNode("B");
g.addEdge("A", "B", 3.5);

if (g.hasEdge("A", "B")) {
    auto weight = g.getEdgeWeight("A", "B");
}
```
