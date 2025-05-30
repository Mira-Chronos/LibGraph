#include <iostream>
#include <vector>
#include <unordered_map>
#include <optional>
#include <algorithm>
#include <unordered_set>
#include <queue>
#include <sstream>

#include "Graph.hpp"

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
	std::cout << directedGraph.toDot() << std::endl;

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

	std::cout << undirectedGraph.toDot() << std::endl;

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
