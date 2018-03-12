#include"Graph.h"
#include"dijkstra.h"
#include"Heap.h"
#include<queue>
#include"LinkQueue.h"
#include<iostream>
#define INF 1000000000
using namespace std;
//#define N 1000000
struct mmp
{
bool operator()(pair<int,int>a,pair<int,int>b)
{
        return a.second>b.second;
}
};
int flag[N];
void dijkstra(Graph *G, int s, set<int> t,int size,float d[],int peg[],float lambda[]){
	int n_num = G->n;
	for (int i = 0; i < n_num; i++)
		if (i == s)
			d[i] = 0;
		else
			d[i] = INF;
	for (int i = 0; i < n_num; i++)
	{
		flag[i] = 0;
		peg[i] = -1;
	}
	//priority_queue<pair<int,int>,vector<pair<int,int>>,mmp>heap;
	Heap heap(n_num);
	for (int i = 0; i < n_num; i++)
		heap.push(i, d[i]);
	//heap.push(make_pair(s,0));
	do{
		//int cur=heap.top().first;
		int cur=heap.pop(); 
		if(flag[cur]==1)continue;
		flag[cur] = 1;
		if (t.find(cur) != t.end())
			{
				size--;
				if(size==0)
					break;
			}
		int size = G->near[cur].size();
		for (int i = 0; i<size; i++){
			int id = G->near[cur][i];	
				Edge* e = &G->incL[id];
				int delt = 0;
				if (flag[e->head] == 0 && d[e->head]>(d[e->tail] +e->weight+lambda[id])){
					d[e->head] = d[e->tail] + e->weight+lambda[id];
					heap.update(e->head, d[e->head]);
					//heap.push(make_pair(e->head, d[e->head]));
					peg[e->head] = id;
				
			}
		}
	} while (!heap.empty());
	
}
void dijcapacity(Graph *G, int s, int t, float d[], int peg[], float lambda[], float capacity[], float demand)
{
	int n_num = G->n;
	for (int i = 0; i < n_num; i++)
		if (i == s)
			d[i] = 0;
		else
			d[i] = INF;
	for (int i = 0; i < n_num; i++)
	{
		flag[i] = 0;
		peg[i] = -1;
	}
	int cur = s;
	Heap heap(n_num);
	for (int i = 0; i < n_num; i++)
		heap.push(i, d[i]);
	do{
		int cur = heap.pop();
		flag[cur] = 1;
		if (cur == t)
			break;
		int size = G->near[cur].size();
		for (int i = 0; i<size; i++){
			int id = G->near[cur][i];
			if (demand<=capacity[id])
			{
				Edge* e = &G->incL[id];
				int delt = 0;
				if (flag[e->head] == 0 && d[e->head]>(d[e->tail] + e->weight + lambda[id])){
					d[e->head] = d[e->tail] + e->weight + lambda[id];
					heap.update(e->head, d[e->head]);
					peg[e->head] = id;
			}
			}
		}
	} while (!heap.empty());
}
void puredijkstra(Graph *G,int s,vector<vector<int>>&d){
	/*int n_num = G->n;
	for (int i = 0; i < n_num; i++)
		if (i == s)
			d[s][i] = 0;
		else
			d[s][i] = INF;*/
	d[s][s]=0;
	queue<int>que;
	que.push(s);
	do{
		int cur=que.front();
		que.pop(); 
		int size = G->near[cur].size();
		for (int i = 0; i<size; i++){
			int id = G->near[cur][i];	
				Edge* e = &G->incL[id];
				if (d[s][e->head]>(d[s][e->tail]+1))
				{
					d[s][e->head] = d[s][e->tail]+1;
					que.push(e->head);
				}
		}
	} while (!que.empty());
}