/**
 * Measures the Closeness Centrality of all the nodes in a connected weighted graph.
 * @author  H M Raine Ahmed, Texas A&M University
 */
#include <iostream>
#include <queue>
#include <fstream>
#include <map>
#include <limits>

using namespace std;
double INFINITY = std::numeric_limits<double>::infinity();

struct Adjacency {
    int node;
    float weight;
};

class Graph {
private:
    int sourceNode;
    int numOfVertices;
    Adjacency **adjList;
    int *nodeList, *degreeList;
    map<int, int> nodeMap;
public:
    Graph(int numOfVertices, int sourceNode, Adjacency **adjacencyList, int *nodeList, int *degreeList,
          map<int, int> nodeMap) {
        this->numOfVertices = numOfVertices;
        this->sourceNode = sourceNode;
        this->adjList = adjacencyList;
        this->nodeList = nodeList;
        this->degreeList = degreeList;
        this->nodeMap = nodeMap;
    }

    int getSourceNode() const {
        return sourceNode;
    }

    void setSourceNode(int sourceNode) {
        Graph::sourceNode = sourceNode;
    }

    int getNumOfVertices() const {
        return numOfVertices;
    }

    void setNumOfVertices(int numOfVertices) {
        Graph::numOfVertices = numOfVertices;
    }

    int *getNodeList() const {
        return nodeList;
    }

    void setNodeList(int *nodeList) {
        Graph::nodeList = nodeList;
    }

    const map<int, int> &getNodeMap() const {
        return nodeMap;
    }

    void setNodeMap(const map<int, int> &nodeMap) {
        Graph::nodeMap = nodeMap;
    }

    int *getDegreeList() const {
        return degreeList;
    }

    void setDegreeList(int *degreeList) {
        Graph::degreeList = degreeList;
    }


    double *shortestPathLength() {
        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> minHeap;
        double *distance = new double[numOfVertices];
        for (int i = 0; i < numOfVertices; i++) {
            distance[i] = INFINITY;
        }

        minHeap.push(make_pair(0, sourceNode));

        // set the distance of the source node to 0
        distance[sourceNode] = 0;

        while (!minHeap.empty()) {
            int u = minHeap.top().second;
            int d = degreeList[u];
            minHeap.pop();

            for (int i = 0; i < d; i++) {
                Adjacency currItem = adjList[u][i];
                int v = currItem.node;
                float weight = currItem.weight;
                if (distance[u] + weight < distance[v]) {
                    distance[v] = distance[u] + weight;
                    minHeap.push(make_pair(distance[v], v));
                }
            }
        }
        return distance;
    }

    Graph readInputGraphFromFile(string filePath) {
        int numberOfVertices = 0;

        // Read adjacency list from input file
        ifstream finput;
        int *nodeList, *degreeList;
        finput.open(filePath);
        int nodeId, degree;
        int nodeIndex = 0;
        int degreeIndex = 0;
        int adjacencyNodeIndex = 0;
        int vertex;
        float weight;
        float var;
        Adjacency **adjList;
        map<int, int> nodeMap;

        if (finput.is_open()) {
            // Read the number of nodes from the first line of the file
            finput >> numberOfVertices;
            // Initialize node list & adjacency list
            nodeList = new int[numberOfVertices];
            degreeList = new int[numberOfVertices];
            adjList = new Adjacency *[numberOfVertices];

            // Read node id and degree, later read the adjacency according to the degree
            while (finput >> nodeId >> degree) {
                // if the node is not already in the map
                bool newNode = nodeMap.find(nodeId) == nodeMap.end();
                if (newNode) { // not found, add the nodeId to the map
                    nodeList[nodeIndex] = nodeId;
                    nodeMap[nodeId] = nodeIndex;
                    nodeIndex++;
                }

                degreeList[degreeIndex] = degree;
                // Dynamically allocate memory for each adjList array element according to the degree
                adjList[adjacencyNodeIndex] = new Adjacency[degree];
                // Read the adjacency upto the degree of the current node
                for (int i = 0; i < degree; i++) {
                    finput >> vertex >> weight >> var;

                    if (nodeMap.find(vertex) == nodeMap.end()) { // not found, add the nodeId to the map
                        nodeList[nodeIndex] = vertex;
                        nodeMap[vertex] = nodeIndex;
                        nodeIndex++;
                    }
                    adjList[adjacencyNodeIndex][i].node = nodeMap[vertex];
                    adjList[adjacencyNodeIndex][i].weight = weight;
                }
                degreeIndex++;
                adjacencyNodeIndex++;
            }
        }
        return Graph(numberOfVertices, sourceNode, adjList, nodeList, degreeList, nodeMap);
    }

};

int main() {
    string filePath = "input.txt";
    Graph g = g.readInputGraphFromFile(filePath);
    int numberOfVertices = g.getNumOfVertices();
    int *nodeList = g.getNodeList();

    for (int i = 0; i < numberOfVertices; i++) {
        int sourceNode = i;
        g.setSourceNode(sourceNode);
        double closeness, sum = 0;
        double *distance = g.shortestPathLength();
        int newNumberOfVertices = numberOfVertices;
        int *degreeList = g.getDegreeList();
        if (degreeList[i] == 0) {
            closeness = INFINITY;
        } else {
            for (int j = 0; j < numberOfVertices; ++j) {
                if (distance[j] == INFINITY) {
                    newNumberOfVertices--;
                    continue;
                }
                sum += distance[j];
            }
            closeness = (newNumberOfVertices - 1) / sum;
        }
        cout << "\n Closeness Centrality of node " << nodeList[i] << ": " << closeness << endl;
    }
    return 0;
}