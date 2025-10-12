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
    int q[4] = {0,0,0,0};
    int green_dir = -1;
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

int nodeId(int r, int c, int C) { return r * C + c; }

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

void simulateCycle(vector<Intersection>& city, const vector<vector<Edge>>& graph,
                   int R, int C,
                   int totalCycleSec, double serviceRate,
                   int &vehiclesArrivedTotal,
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

    for (int i = 0; i < n; ++i) {
        Intersection &I = city[i];
        vector<int> greenTimes = allocateGreenTimes(I, totalCycleSec);
        int best = 0;
        for (int d = 1; d < 4; ++d) if (greenTimes[d] > greenTimes[best]) best = d;
        I.green_dir = best;

        for (int d = 0; d < 4; ++d) {
            int canServe = (int)floor(serviceRate * greenTimes[d] + 1e-9);
            int served = min(canServe, I.q[d]);
            I.q[d] -= served;
            totalVehiclesServed += served;
        }

        cumulativeQueueSum += (I.q[0] + I.q[1] + I.q[2] + I.q[3]);
    }
}
int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    srand(time(nullptr));

    int R = 2, C = 2;
    cout << "Enter grid rows R (default 2): ";
    string tmp; getline(cin, tmp);
    if(!tmp.empty()) R = stoi(tmp);
    cout << "Enter grid cols C (default 2): ";
    getline(cin, tmp);
    if(!tmp.empty()) C = stoi(tmp);

    int n = R * C;
    vector<vector<Edge>> graph;
    buildGridGraph(R, C, graph);

    vector<Intersection> city(n);
    for(int i=0; i<n; i++) city[i].id = i;

    // Random initial traffic
    for(int i=0; i<n; i++)
        for(int d=0; d<4; d++)
            city[i].q[d] = rand() % 20;

    cout << "\nInitial network state:\n";
    printNetworkState(city, R, C, 0);

    int totalCycleSec = 30;
    cout << "Enter cycle time per intersection in seconds (default 30): ";
    getline(cin, tmp);
    if(!tmp.empty()) totalCycleSec = stoi(tmp);

    double serviceRate = 0.5;
    cout << "Enter service rate (vehicles per second when green, default 0.5): ";
    getline(cin, tmp);
    if(!tmp.empty()) serviceRate = stod(tmp);

    // Metrics
    int vehiclesArrivedTotal = 0;
    long long cumulativeQueueSum = 0;
    long long totalVehiclesServed = 0;

    // Simulate one cycle
    simulateCycle(city, graph, R, C, totalCycleSec, serviceRate,
                  vehiclesArrivedTotal, cumulativeQueueSum, totalVehiclesServed);

    cout << "\nAfter one cycle:\n";
    printNetworkState(city, R, C, 1);

    cout << "Vehicles arrived: " << vehiclesArrivedTotal << "\n";
    cout << "Vehicles served: " << totalVehiclesServed << "\n";

    return 0;
}