#include <iostream>

#include "every_thing.h"
//
//
//#include <iostream>
//#include <vector>
//#include <cstring>
//#include <queue>
//
//using namespace std;
//
//const int N = 10010, INF = 1e8;
//
//struct Edge {
//    int v, w;
//    Edge(int _v, int _w) : v(_v), w(_w) {}
//};
//
//vector<Edge> edges[N];
//int dist[N], pre[N];
//bool st[N];
//vector<int> route; //机器人编号、路径
//
//// 添加边的函数
//void addEdge(int u, int v, int w) {
//    edges[u].push_back(Edge(v, w));
//    edges[v].push_back(Edge(u, w));
//}
//
//// Bellman-Ford算法求最短路径
//bool bellman_ford(int s, int t) {
//    memset(dist, 0x3f, sizeof(dist));
//    dist[s] = 0;
//    queue<int> q;
//    q.push(s);
//    st[s] = true;
//    while (!q.empty()) {
//        int u = q.front();
//        q.pop();
//        st[u] = false;
//        for (auto& e : edges[u]) {
//            int v = e.v, w = e.w;
//            if (dist[v] > dist[u] + w) {
//                dist[v] = dist[u] + w;
//                pre[v] = u;
//                if (!st[v]) {
//                    q.push(v);
//                    st[v] = true;
//                }
//            }
//        }
//    }
//    return dist[t] < INF;
//}
//
//// 输出最小路径
//void generate_path(int s, int t) {
//    if (s == t) {
//        route.push_back(s);
//        return;
//    }
//    generate_path(s, pre[t]);
//    route.push_back(t);
//}
//
//int main() {
//    int n = 5, m = 7;
//    addEdge(1, 2, 2);
//    addEdge(1, 3, 3);
//    addEdge(2, 3, 1);
//    addEdge(2, 4, 5);
//    addEdge(3, 4, 2);
//    addEdge(3, 5, 4);
//    addEdge(4, 5, 1);
//    int s = 4, t = 1;
//    //调用bellman_ford(s, t)
//    if (bellman_ford(s, t)) {
//        for(int i = 0; i < 10; ++i){
//        }
//        cerr << "Value of the best route from " << s << " to " << t << " is " << dist[t] << endl;
//        generate_path(s, t);
//        cerr << "Route is: ";
//        for (auto it : route) {
//            cerr << it << " ";
//        }
//    } else {
//        cerr << "No path from " << s << " to " << t << endl;
//    }
//    return 0;
//}
//
//
//
int main() {

    auto* distributor = new Distributor();
    distributor->run();

    return 0;
}
