#include"Graph.h"
using namespace std;
void BFS(Graph *G, int s, int t, float d[], int peg[], float demand, float capacity[],float rv);
void BFSO(Graph *G, int s, int t,float d[], int peg[], float demand, float capacity[],float rv,vector<vector<int>>&mind);