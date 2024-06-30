//Nir Ohayon - 206542953
//Raz kalfon - 213006083

#include "std_lib_facilities.h"
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <string>
#include <sys/wait.h>
#include <queue>
#include <set>
#include <pthread.h>

class Graph {
private:
    unordered_map<int, vector<int>> adjacencyList;
    deque<vector<int>> requestCache; // Cache to store the last 10 requests with routes

public:
    // Function to add an edge between two nodes in the graph
    void addEdge(int u, int v) {
        adjacencyList[u].push_back(v);
        adjacencyList[v].push_back(u);
    }
    // Get the adjacency list of the graph
    const unordered_map<int, vector<int>>& getAdjacencyList() const {
        return adjacencyList;
    }

    // Print the adjacency list of the graph. away to check my code.

    // void GraphPrint() const {
    //     for (const auto& pair : adjacencyList) {
    //         int node = pair.first;
    //         const auto& neighbors = pair.second;

    //         cout << node << ": ";
    //         for (int neighbor : neighbors) {
    //             cout << neighbor << " ";
    //         }
    //         cout << endl;
    //     }
    // }


    // Function that add request to cache.
    void addRequestToCache(int source, int destination, const vector<int>& shortestPath) {
        if (requestCache.size() >= 10) {
            cout << "Cache is full. Removing the oldest request." << endl;
            requestCache.pop_front();
        }
    
        vector<int> route;
        route.push_back(source);
        route.push_back(destination);
        // Append the shortest path to the route
        route.insert(route.end(), shortestPath.begin(), shortestPath.end());
        requestCache.push_back(route);
    }
    
    //away to check my code- Print the Cache 

    // void printCache() const {
    //     cout << "Cache size: " << requestCache.size() << endl;
    //     cout << "Printing cache:" << endl;
        
    //     for (const auto& route : requestCache) {
    //         cout << "Route: ";
    //         for (int node : route) {
    //             cout << node << " ";
    //         }
    //         cout << endl;
    //     }
    // }

    const deque<vector<int>>& getRequestCache() const {
        return requestCache;
    }
};

// Function to do BFS on the graph to find the shortest path
vector<int> BFS(const Graph& graph, int source, int destination) {
    unordered_map<int, int> parent;
    queue<int> q;
    set<int> visited;

    q.push(source);
    visited.insert(source);
    parent[source] = -1;

    bool found = false;

    while (!q.empty()) {
        int current = q.front();
        q.pop();

        if (current == destination) {
            found = true;
            break;
        }

        for (int neighbor : graph.getAdjacencyList().at(current)) {
            if (visited.find(neighbor) == visited.end()) {
                q.push(neighbor);
                visited.insert(neighbor);
                parent[neighbor] = current;
            }
        }
    }

    if (!found) {
        return vector<int>();
    }

    vector<int> path;
    int current = destination;
    while (current != -1) {
        path.push_back(current);
        current = parent[current];
    }

    reverse(path.begin(), path.end());

    return path;
}

// Function to load graph data from a file into the provided graph object
void loadGraphFromFile(const string& filename, Graph& graph) {
    ifstream file(filename);
    if (!file.is_open()) {
        error("Error: Unable to open file");
        return;
    }

    string line;
    while (getline(file, line)) {
        istringstream iss(line);
        
        int u,v;
        if (!(iss >> u >> v)) {
            error("Error: Invalid line format in file");
            continue;
        }
        graph.addEdge(u, v);
    }

    file.close();
}
// Mutex for thread
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

struct ThreadArgs {
    Graph* graph;
    int fd;
};
// Thread function to handle client requests as we learned on the lecture
void* threadFunc(void* args) {
    ThreadArgs* threadArgs = static_cast<ThreadArgs*>(args);

    char buffer[256] = {0};
    read(threadArgs->fd, buffer, 256);

    int sourceNode, destinationNode;
    sscanf(buffer, "%d %d", &sourceNode, &destinationNode);

    pthread_mutex_lock(&mutex);

    Graph* graph = threadArgs->graph;

    bool inCache = false;
    auto& cache = graph->getRequestCache();
    for (const auto& route : cache) {
        if (route[0] == sourceNode && route[1] == destinationNode) {
            inCache = true;
            cout << "Route from " << sourceNode << " to " << destinationNode << " found in cache:" << endl;
            // // Print the cached route
            // for (size_t i = 2; i < route.size(); ++i) {
            //     cout << route[i] << " ";
            // }
            // cout << endl;
            // Send the cached result back to the client
            string cachedResult;
            for (int vertex : route) {
                cachedResult += to_string(vertex) + " ";
            }
            write(threadArgs->fd, cachedResult.c_str(), cachedResult.size());
            break;
        }
    }

    // After calculating the result
    if (!inCache) {
        vector<int> shortestPath = BFS(*graph, sourceNode, destinationNode);
        graph->addRequestToCache(sourceNode, destinationNode, shortestPath);

        if (shortestPath.empty() || shortestPath[0] != sourceNode || shortestPath.back() != destinationNode) {
            string noPathMessage = "No path exists between " + to_string(sourceNode) + " and " + to_string(destinationNode);
            write(threadArgs->fd, noPathMessage.c_str(), noPathMessage.size());
        } else {
            string result;
            for (int vertex : shortestPath) {
                result += to_string(vertex) + " ";
            }
            // Send the result back to the client
            write(threadArgs->fd, result.c_str(), result.size());
        }
    }

    pthread_mutex_unlock(&mutex);
    close(threadArgs->fd);
    delete threadArgs;
    pthread_exit(NULL);
}

int main(int argc, char **argv) {
    if (argc != 3) {
        error("Please enter 2 elements <filename> <port>");
    }

    Graph graph;
    loadGraphFromFile(argv[1], graph);

    const string filename = argv[1];
    const int port = atoi(argv[2]);

    int fd=socket(AF_INET,SOCK_STREAM,0);
    sockaddr_in addr={0};
    addr.sin_family=AF_INET;
    addr.sin_addr.s_addr=inet_addr("127.0.0.1");
    addr.sin_port=htons(port);

    bind(fd,(sockaddr*)&addr,sizeof(addr));

    listen(fd,5);
    
    //print the listening port to make sure.
    cout << "Server listening on port " << port << endl;

    for (;;) {
        int fd2 = accept(fd, NULL, NULL);
        if (fd2 == -1) {
            error("Accept failed");
            continue;
        }

        ThreadArgs* args = new ThreadArgs{&graph, fd2};
        pthread_t thread;
        if (pthread_create(&thread, NULL, threadFunc, args) != 0) {
            error("Thread creation failed");
            close(fd2);
            delete args;
            continue;
        }

        if (pthread_detach(thread) != 0) {
            error("Thread detachment failed");
            close(fd2);
        }
    }

    close(fd);
}
