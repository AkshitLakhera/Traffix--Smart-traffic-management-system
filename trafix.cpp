// smart_traffic.cpp
// Smart Traffic Network Simulation (Grid graph, adaptive local signals, ambulance priority)
// Compile: g++ -std=c++17 smart_traffic.cpp -O2 -o smart_traffic
// Run: ./smart_traffic

#include <vector>
#include<string>
#include<algorithm>
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
// totalCycleSec is total seconds allocated per cycle for this intersection
// serviceRate: vehicles cleared per second when that direction is green
// Returns vector<int> greenTimeSec(4)
vector<int> allocateGreenTimes(const Intersection &I, int totalCycleSec) {
    int total = I.q[0] + I.q[1] + I.q[2] + I.q[3];
    vector<int> times(4, 0);
    if (total == 0) {
        // distribute equally if empty
        for (int i = 0; i < 4; ++i) times[i] = totalCycleSec / 4;
        // remainder to first
        times[0] += totalCycleSec % 4;
        return times;
    }
    // proportional allocation, ensure sum == totalCycleSec
    int assigned = 0;
    for (int i = 0; i < 4; ++i) {
        double ratio = (double)I.q[i] / total;
        times[i] = max(1, (int)round(ratio * totalCycleSec)); // give at least 1 second when there's traffic
        assigned += times[i];
    }
    // fix rounding errors
    while (assigned > totalCycleSec) {
        // subtract one from direction with smallest queue but >1
        int idx = -1, bestQ = INT_MAX;
        for (int i = 0; i < 4; ++i) if (times[i] > 1 && I.q[i] < bestQ) { idx = i; bestQ = I.q[i]; }
        if (idx == -1) break;
        times[idx]--; assigned--;
    }
    while (assigned < totalCycleSec) {
        // add one to direction with largest queue
        int idx = -1, bestQ = -1;
        for (int i = 0; i < 4; ++i) if (I.q[i] > bestQ) { idx = i; bestQ = I.q[i]; }
        times[idx]++; assigned++;
    }
    return times;
}

// Simulate one cycle for all intersections
// If ambulancePath is non-empty, intersections on that path will have ambulance_override set for the right outbound lane.
// R,C used to compute direction of movement along path
void simulateCycle(vector<Intersection>& city, const vector<vector<Edge>>& graph,
                   int R, int C,
                   int totalCycleSec, double serviceRate,
                   vector<int> ambulancePath, int &vehiclesArrivedTotal,
                   long long &cumulativeQueueSum, long long &totalVehiclesServed)
{
    int n = city.size();
    // Step A: New arrivals (random). We'll use arrivals in range [0, maxArrivalPerLane]
    int maxArrivalPerLane = 5; // you can tweak
    for (int i = 0; i < n; ++i) {
        for (int d = 0; d < 4; ++d) {
            int arr = rand() % (maxArrivalPerLane + 1);
            city[i].q[d] += arr;
            vehiclesArrivedTotal += arr;
        }
    }

    // Setup ambulance overrides: map node->outboundDir = true
    for (int i = 0; i < n; ++i)
        for (int d = 0; d < 4; ++d)
            city[i].ambulance_override[d] = false;

    if (!ambulancePath.empty()) {
        // For each consecutive pair in path, set override at the from-node for the correct outbound direction
        for (int idx = 0; idx + 1 < (int)ambulancePath.size(); ++idx) {
            int u = ambulancePath[idx];
            int v = ambulancePath[idx+1];
            // compute (r,c) for u and v
            int ur = u / C, uc = u % C;
            int vr = v / C, vc = v % C;
            int dir = -1;
            if (vr == ur -1 && vc == uc) dir = 0; // N
            else if (vr == ur +1 && vc == uc) dir = 1; // S
            else if (vr == ur && vc == uc +1) dir = 2; // E
            else if (vr == ur && vc == uc -1) dir = 3; // W
            if (dir >= 0) {
                city[u].ambulance_override[dir] = true;
            }
        }
    }

    // Step B: For each intersection, allocate green times and serve vehicles
    for (int i = 0; i < n; ++i) {
        Intersection &I = city[i];

        // If ambulance override present: ensure the direction that ambulance uses gets most of the cycle or entire cycle
        bool hasOverride = false;
        for (int d = 0; d < 4; ++d) if (I.ambulance_override[d]) hasOverride = true;
        vector<int> greenTimes = allocateGreenTimes(I, totalCycleSec);

        if (hasOverride) {
            // Ensure overridden directions get priority: give them max of cycle until queue in that direction reduces (simple policy)
            int overrideTotal = 0;
            for (int d = 0; d < 4; ++d) if (I.ambulance_override[d]) overrideTotal += greenTimes[d];

            // Strategy: give all cycle seconds to first overridden direction (so ambulance passes quickly)
            int giveDir = -1;
            for (int d = 0; d < 4; ++d) if (I.ambulance_override[d]) { giveDir = d; break; }
            if (giveDir >= 0) {
                for (int d = 0; d < 4; ++d) greenTimes[d] = 0;
                greenTimes[giveDir] = totalCycleSec;
                I.green_dir = giveDir;
            }
        } else {
            // choose the direction with maximum green time for printing
            int best = 0;
            for (int d = 1; d < 4; ++d) if (greenTimes[d] > greenTimes[best]) best = d;
            I.green_dir = best;
        }

        // Serve vehicles per direction
        for (int d = 0; d < 4; ++d) {
            int serveSec = greenTimes[d];
            int canServe = (int)floor(serviceRate * serveSec + 1e-9);
            int served = min(canServe, I.q[d]);
            I.q[d] -= served;
            totalVehiclesServed += served;
        }

        // accumulate queue sum metric
        cumulativeQueueSum += (I.q[0] + I.q[1] + I.q[2] + I.q[3]);
    }
}

// Utility to print path nicely
void printPath(const vector<int>& path, int R, int C) {
    if (path.empty()) {
        cout << "No path found.\n";
        return;
    }
    cout << "Shortest path nodes: ";
    for (int i = 0; i < (int)path.size(); ++i) {
        cout << path[i];
        if (i + 1 < (int)path.size()) cout << " -> ";
    }
    cout << "\nGrid coords: ";
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

    // Create intersections
    vector<Intersection> city(n);
    for (int i = 0; i < n; ++i) city[i].id = i;

    // initial random traffic
    for (int i = 0; i < n; ++i) {
        for (int d = 0; d < 4; ++d) city[i].q[d] = rand() % 20; // initial 0-19 vehicles
    }

    cout << "Grid built with " << R << " x " << C << " = " << n << " intersections.\n";
    cout << "Each intersection has 4 lanes: N S E W.\n";

    // Simulation parameters
    int totalCycles = 10;
    cout << "Enter number of simulation cycles (default 10): ";
    getline(cin, tmp);
    if (!tmp.empty()) totalCycles = stoi(tmp);

    int totalCycleSec = 30; // seconds per intersection cycle
    cout << "Enter cycle time per intersection in seconds (default 30): ";
    getline(cin, tmp);
    if (!tmp.empty()) totalCycleSec = stoi(tmp);

    double serviceRate = 0.5; // vehicles per second when a lane is green
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

    // Metrics
    int vehiclesArrivedTotal = 0;
    long long cumulativeQueueSum = 0; // sum of all queue lengths across cycles (for averaging)
    long long totalVehiclesServed = 0;

    for (int cycle = 1; cycle <= totalCycles; ++cycle) {
        cout << "\n----- SIMULATION CYCLE " << cycle << " -----\n";

        // Ambulance path to apply for this cycle
        vector<int> ambulancePath;

        if (ambulance_enabled && cycle == amb_cycle) {
            cout << "\n*** Ambulance arrives at cycle " << cycle << " ***\n";
            cout << "Calculating shortest path from " << amb_src << " to " << amb_dest << "...\n";
            ambulancePath = dijkstraPath(amb_src, amb_dest, graph);
            printPath(ambulancePath, R, C);
            if (!ambulancePath.empty()) {
                cout << "Will override signals along this path for this cycle to allow ambulance clearance.\n";
            } else {
                cout << "No path found. Ambulance cannot be routed.\n";
            }
        }

        // Print current network state before this cycle (queues, etc.)
        printNetworkState(city, R, C, cycle);

        // Simulate one cycle; if ambulancePath non-empty, its overrides will be applied
        simulateCycle(city, graph, R, C, totalCycleSec, serviceRate,
                      ambulancePath, vehiclesArrivedTotal, cumulativeQueueSum, totalVehiclesServed);

        // Print state after cycle
        cout << "\nAfter cycle " << cycle << " (post-serving):\n";
        printNetworkState(city, R, C, cycle);

        // Print summary small stats
        cout << "Vehicles arrived so far: " << vehiclesArrivedTotal << "\n";
        cout << "Total vehicles served so far: " << totalVehiclesServed << "\n";
    }

    // Final metrics
    double avgQueueLengthPerCyclePerNode = 0.0;
    if (totalCycles > 0) avgQueueLengthPerCyclePerNode = (double)cumulativeQueueSum / (totalCycles * n);
    cout << "\n=== Simulation Complete ===\n";
    cout << "Total cycles: " << totalCycles << "\n";
    cout << "Total vehicles arrived (approx): " << vehiclesArrivedTotal << "\n";
    cout << "Total vehicles served (approx): " << totalVehiclesServed << "\n";
    cout << "Average queue length per node per cycle: " << fixed << setprecision(2) << avgQueueLengthPerCyclePerNode << "\n";
    cout << "You can tweak grid size, arrivals, service rate, cycle time for experiments.\n";

    cout << "\nNotes & Extensions:\n";
    cout << " - This simulation uses a grid graph; you can replace graph builder with custom graph input if needed.\n";
    cout << " - Ambulance override gives full cycle to the outbound lane at each node on path for quick clearance.\n";
    cout << " - For presentation, show a run with ambulance and without to compare average queue lengths.\n";

    return 0;
}
