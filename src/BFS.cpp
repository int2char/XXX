#include"Graph.h"
#include"dijkstra.h"
#include"Heap.h"
#include<queue>
#include"math.h"
#define INF 100000
#define N 10000
int flag2[N];
void BFS(Graph *G, int s, int t, float d[], int peg[], float demand, float capacity[],float rv){
	int n_num = G->n;
	for (int i = 0; i < n_num; i++)
		if (i == s)
			d[i] = 0;
		else
			d[i] = INF;
	for (int i = 0; i < n_num; i++)
		peg[i] = -1;
	int cur = s;
	queue<pair<int,int>>que;
	que.push(make_pair(s,0));
	do{
		int cur=que.front().first;
		float dd=que.front().second;
		que.pop();
		if (cur == t)
			break;
		if(demand/pow(dd,0.5)<rv)break;
		int size = G->near[cur].size();
		for (int i = size-1; i>=0; i--)
		{ 
			int id = G->near[cur][i];
			if (capacity[id]>=demand)
			{
				int head = G->incL[id].head;
				int tail = G->incL[id].tail;
				if (flag2[head] == 0 && d[head] > d[tail] + 1)
				{
					d[head] = d[tail] + 1;
					peg[head] = id;
					que.push(make_pair(head,dd++));
				}
			}
		}
	} while (!que.empty());
}