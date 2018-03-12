#include"Graph.h"
#include"set"
using namespace std;
void dijkstra(Graph *G, int s, set<int> t, int size,float d[], int peg[], float lambda[]);
void dijcapacity(Graph *G, int s, int t, float d[], int peg[], float lambda[],float capacity[],float demand);
void puredijkstra(Graph *G,int s,vector<vector<int>>&d);