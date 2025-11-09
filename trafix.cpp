// smart_traffic.cpp
// Smart Traffic Network Simulation (Grid graph, adaptive local signals, ambulance priority)
// Compile: g++ -std=c++17 smart_traffic.cpp -o smart_traffic
// Run: ./smart_traffic

#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <algorithm>
#include <ctime>
#include <cstdlib>
#include <cmath>
#include <iomanip>
#include <climits>

using namespace std;
using ll = long long;

struct Edge { int to; int w; };
struct Intersection {
    int id;
    // queue length for each direction: 0=N,1=S,2=E,3=W
    int q[4] = {0,0,0,0};
    // which direction currently green (for printing) - -1 = none (during cycle output)
    int green_dir = -1;
    // if overridden by ambulance this cycle: set of directions forced green
    bool ambulance_override[4] = {false,false,false,false};
};

int dr[4] = {-1, 1, 0, 0}; // N S E W
int dc[4] = {0, 0, 1, -1};
string dirName(int d) {
    if (d == 0) return "N";
    if (d == 1) return "S";
    if (d == 2) return "E";
    return "W";
}

// Convert (r,c) to node id
int nodeId(int r, int c, int C) { return r * C + c; }

// Dijkstra to find shortest path on grid graph
vector<int> dijkstraPath(int src, int dest, const vector<vector<Edge>>& graph) {
    int n = graph.size();
    const int INF = 1e9;
    vector<int> dist(n, INF), parent(n, -1);
    dist[src] = 0;
    priority_queue<pair<int,int>, vector<pair<int,int>>, greater<pair<int,int>>> pq;
    pq.push({0, src});
    while (!pq.empty()) {
        auto [d,u] = pq.top(); pq.pop();
        if (d != dist[u]) continue;
        if (u == dest) break;
        for (auto &e : graph[u]) {
            int v = e.to;
            if (dist[u] + e.w < dist[v]) {
                dist[v] = dist[u] + e.w;
                parent[v] = u;
                pq.push({dist[v], v});
            }
        }
    }
    vector<int> path;
    if (dist[dest] == INF) return path;
    for (int v = dest; v != -1; v = parent[v]) path.push_back(v);
    reverse(path.begin(), path.end());
    return path;
}

// Find least congested path: uses total queue sum as edge weight
vector<int> dijkstraCongestionPath(int src, int dest, const vector<vector<Edge>>& graph, const vector<Intersection>& city) {
    int n = graph.size();
    const int INF = 1e9;
    vector<int> dist(n, INF), parent(n, -1);
    dist[src] = 0;
    priority_queue<pair<int,int>, vector<pair<int,int>>, greater<pair<int,int>>> pq;
    pq.push({0, src});
    while (!pq.empty()) {
        auto [d,u] = pq.top(); pq.pop();
        if (d != dist[u]) continue;
        if (u == dest) break;
        for (auto &e : graph[u]) {
            int v = e.to;
            // Weight = 1 (base distance) + congestion (total queue at v)
            int congestion = city[v].q[0] + city[v].q[1] + city[v].q[2] + city[v].q[3];
            int edgeWeight = 1 + (congestion / 5); // divide by 5 to scale congestion reasonably
            if (dist[u] + edgeWeight < dist[v]) {
                dist[v] = dist[u] + edgeWeight;
                parent[v] = u;
                pq.push({dist[v], v});
            }
        }
    }
    vector<int> path;
    if (dist[dest] == INF) return path;
    for (int v = dest; v != -1; v = parent[v]) path.push_back(v);
    reverse(path.begin(), path.end());
    return path;
}

// Build grid graph: R rows x C cols, edges between 4-neighbors with weight = 1
void buildGridGraph(int R, int C, vector<vector<Edge>>& graph) {
    int n = R * C;
    graph.assign(n, {});
    for (int r = 0; r < R; ++r) {
        for (int c = 0; c < C; ++c) {
            int u = nodeId(r,c,C);
            for (int d = 0; d < 4; ++d) {
                int nr = r + dr[d], nc = c + dc[d];
                if (nr >= 0 && nr < R && nc >= 0 && nc < C) {
                    int v = nodeId(nr,nc,C);
                    graph[u].push_back({v, 1});
                }
            }
        }
    }
}

// Print a simple visualization of the intersections and their queues
void printNetworkState(const vector<Intersection>& city, int R, int C, int cycle) {
    cout << "\n=== Cycle " << cycle << " Network State ===\n";
    for (int r = 0; r < R; ++r) {
        for (int c = 0; c < C; ++c) {
            int id = nodeId(r,c,C);
            const Intersection &I = city[id];
            cout << "[Node " << id << "]";
            cout << " (N:" << I.q[0] << " S:" << I.q[1] << " E:" << I.q[2] << " W:" << I.q[3] << ")";
            if (I.green_dir >= 0) cout << " G:" << dirName(I.green_dir);
            cout << "  ";
        }
        cout << "\n";
    }
    cout << "==============================\n";
}

// Decide green time proportionally for each direction at a node
vector<int> allocateGreenTimes(const Intersection &I, int totalCycleSec) {
    int total = I.q[0] + I.q[1] + I.q[2] + I.q[3];
    vector<int> times(4, 0);
    if (total == 0) {
        for (int i = 0; i < 4; ++i) times[i] = totalCycleSec / 4;
        times[0] += totalCycleSec % 4;
        return times;
    }
    int assigned = 0;
    for (int i = 0; i < 4; ++i) {
        double ratio = (double)I.q[i] / total;
        times[i] = max(1, (int)round(ratio * totalCycleSec));
        assigned += times[i];
    }
    while (assigned > totalCycleSec) {
        int idx = -1, bestQ = INT_MAX;
        for (int i = 0; i < 4; ++i) if (times[i] > 1 && I.q[i] < bestQ) { idx = i; bestQ = I.q[i]; }
        if (idx == -1) break;
        times[idx]--; assigned--;
    }
    while (assigned < totalCycleSec) {
        int idx = -1, bestQ = -1;
        for (int i = 0; i < 4; ++i) if (I.q[i] > bestQ) { idx = i; bestQ = I.q[i]; }
        times[idx]++; assigned++;
    }
    return times;
}

// Simulate one cycle for all intersections
void simulateCycle(vector<Intersection>& city, const vector<vector<Edge>>& graph,
                   int R, int C,
                   int totalCycleSec, double serviceRate,
                   vector<int> ambulancePath, int &vehiclesArrivedTotal,
                   long long &cumulativeQueueSum, long long &totalVehiclesServed)
{
    int n = city.size();
    int maxArrivalPerLane = 5;
    for (int i = 0; i < n; ++i) {
        for (int d = 0; d < 4; ++d) {
            int arr = rand() % (maxArrivalPerLane + 1);
            city[i].q[d] += arr;
            vehiclesArrivedTotal += arr;
        }
    }

    for (int i = 0; i < n; ++i)
        for (int d = 0; d < 4; ++d)
            city[i].ambulance_override[d] = false;

    if (!ambulancePath.empty()) {
        for (int idx = 0; idx + 1 < (int)ambulancePath.size(); ++idx) {
            int u = ambulancePath[idx];
            int v = ambulancePath[idx+1];
            int ur = u / C, uc = u % C;
            int vr = v / C, vc = v % C;
            int dir = -1;
            if (vr == ur -1 && vc == uc) dir = 0;
            else if (vr == ur +1 && vc == uc) dir = 1;
            else if (vr == ur && vc == uc +1) dir = 2;
            else if (vr == ur && vc == uc -1) dir = 3;
            if (dir >= 0) {
                city[u].ambulance_override[dir] = true;
            }
        }
    }

    for (int i = 0; i < n; ++i) {
        Intersection &I = city[i];

        bool hasOverride = false;
        for (int d = 0; d < 4; ++d) if (I.ambulance_override[d]) hasOverride = true;
        vector<int> greenTimes = allocateGreenTimes(I, totalCycleSec);

        if (hasOverride) {
            int giveDir = -1;
            for (int d = 0; d < 4; ++d) if (I.ambulance_override[d]) { giveDir = d; break; }
            if (giveDir >= 0) {
                for (int d = 0; d < 4; ++d) greenTimes[d] = 0;
                greenTimes[giveDir] = totalCycleSec;
                I.green_dir = giveDir;
            }
        } else {
            int best = 0;
            for (int d = 1; d < 4; ++d) if (greenTimes[d] > greenTimes[best]) best = d;
            I.green_dir = best;
        }

        for (int d = 0; d < 4; ++d) {
            int serveSec = greenTimes[d];
            int canServe = (int)floor(serviceRate * serveSec + 1e-9);
            int served = min(canServe, I.q[d]);
            I.q[d] -= served;
            totalVehiclesServed += served;
        }

        cumulativeQueueSum += (I.q[0] + I.q[1] + I.q[2] + I.q[3]);
    }
}

// Utility to print path nicely
void printPath(const vector<int>& path, const string& label, int R, int C) {
    if (path.empty()) {
        cout << label << " No path found.\n";
        return;
    }
    cout << label << " Nodes: ";
    for (int i = 0; i < (int)path.size(); ++i) {
        cout << path[i];
        if (i + 1 < (int)path.size()) cout << " -> ";
    }
    cout << " | Coords: ";
    for (int i = 0; i < (int)path.size(); ++i) {
        int r = path[i] / C, c = path[i] % C;
        cout << "(" << r << "," << c << ")";
        if (i + 1 < (int)path.size()) cout << " -> ";
    }
    cout << "\n";
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    srand((unsigned)time(nullptr));

    cout << "Smart Traffic Management (Grid + Ambulance Priority)\n";
    cout << "---------------------------------------------------\n";

    int R = 2, C = 2;
    cout << "Enter grid rows R (default 2): ";
    string tmp; getline(cin, tmp);
    if (!tmp.empty()) R = stoi(tmp);
    cout << "Enter grid cols C (default 2): ";
    getline(cin, tmp);
    if (!tmp.empty()) C = stoi(tmp);
    int n = R * C;
    vector<vector<Edge>> graph;
    buildGridGraph(R, C, graph);

    vector<Intersection> city(n);
    for (int i = 0; i < n; ++i) city[i].id = i;

    for (int i = 0; i < n; ++i) {
        for (int d = 0; d < 4; ++d) city[i].q[d] = rand() % 20;
    }

    cout << "Grid built with " << R << " x " << C << " = " << n << " intersections.\n";
    cout << "Each intersection has 4 lanes: N S E W.\n";

    int totalCycles = 10;
    cout << "Enter number of simulation cycles (default 10): ";
    getline(cin, tmp);
    if (!tmp.empty()) totalCycles = stoi(tmp);

    int totalCycleSec = 30;
    cout << "Enter cycle time per intersection in seconds (default 30): ";
    getline(cin, tmp);
    if (!tmp.empty()) totalCycleSec = stoi(tmp);

    double serviceRate = 0.5;
    cout << "Enter service rate (vehicles per second when green, default 0.5): ";
    getline(cin, tmp);
    if (!tmp.empty()) serviceRate = stod(tmp);

    cout << "\nDo you want to trigger an ambulance during the simulation? (y/n, default n): ";
    getline(cin, tmp);
    bool ambulance_enabled = false;
    if (!tmp.empty() && (tmp[0] == 'y' || tmp[0] == 'Y')) ambulance_enabled = true;

    int amb_cycle = 1, amb_src = 0, amb_dest = n-1;
    if (ambulance_enabled) {
        cout << "Enter cycle number when ambulance appears (1-based, within simulation): ";
        getline(cin, tmp);
        if (!tmp.empty()) amb_cycle = stoi(tmp);
        cout << "Enter ambulance source node id (0 to " << n-1 << ") (default 0): ";
        getline(cin, tmp);
        if (!tmp.empty()) amb_src = stoi(tmp);
        cout << "Enter ambulance destination node id (0 to " << n-1 << ") (default " << n-1 << "): ";
        getline(cin, tmp);
        if (!tmp.empty()) amb_dest = stoi(tmp);
    }

    cout << "\nStarting simulation...\n";

    int vehiclesArrivedTotal = 0;
    long long cumulativeQueueSum = 0;
    long long totalVehiclesServed = 0;

    for (int cycle = 1; cycle <= totalCycles; ++cycle) {
        cout << "\n----- SIMULATION CYCLE " << cycle << " -----\n";

        vector<int> ambulancePath;

        if (ambulance_enabled && cycle == amb_cycle) {
            cout << "\n*** Ambulance arrives at cycle " << cycle << " ***\n";
            cout << "Source: " << amb_src << " | Destination: " << amb_dest << "\n";
            
            // Shortest path
            vector<int> shortestPath = dijkstraPath(amb_src, amb_dest, graph);
            printPath(shortestPath, "SHORTEST PATH:", R, C);
            
            // Least congested path
            vector<int> congestionPath = dijkstraCongestionPath(amb_src, amb_dest, graph, city);
            printPath(congestionPath, "LEAST CONGESTED PATH:", R, C);
            
            // Decide which to use (use shortest by default, but show both)
            cout << "\nUsing SHORTEST PATH for ambulance routing this cycle.\n";
            ambulancePath = shortestPath;
        }

        printNetworkState(city, R, C, cycle);

        simulateCycle(city, graph, R, C, totalCycleSec, serviceRate,
                      ambulancePath, vehiclesArrivedTotal, cumulativeQueueSum, totalVehiclesServed);

        cout << "\nAfter cycle " << cycle << " (post-serving):\n";
        printNetworkState(city, R, C, cycle);

        cout << "Vehicles arrived so far: " << vehiclesArrivedTotal << "\n";
        cout << "Total vehicles served so far: " << totalVehiclesServed << "\n";
    }

    double avgQueueLengthPerCyclePerNode = 0.0;
    if (totalCycles > 0) avgQueueLengthPerCyclePerNode = (double)cumulativeQueueSum / (totalCycles * n);
    cout << "\n=== Simulation Complete ===\n";
    cout << "Total cycles: " << totalCycles << "\n";
    cout << "Total vehicles arrived (approx): " << vehiclesArrivedTotal << "\n";
    cout << "Total vehicles served (approx): " << totalVehiclesServed << "\n";
    cout << "Average queue length per node per cycle: " << fixed << setprecision(2) << avgQueueLengthPerCyclePerNode << "\n";

    cout << "\nFeatures:\n";
    cout << " - Calculates SHORTEST PATH (distance-based)\n";
    cout << " - Calculates LEAST CONGESTED PATH (queue-aware)\n";
    cout << " - Both paths shown when ambulance arrives\n";
    cout << " - Currently using shortest path; modify to compare or switch based on congestion levels.\n";

    return 0;
}