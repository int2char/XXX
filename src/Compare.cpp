#include"Compare.h"
#include "GraphPath.h"
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include<vector>
#include<algorithm>
#include <utility>
#include <time.h>
#include<math.h>
#include"service.h"
#include"dijkstra.h"
#define INF 1000000
#define N 10000
using namespace std;
float dt[N];
float lambda[N];
/*int pre[N];
int peg[N];
int flag[N + 10];
int d[N];
class Heap{
private:
	Edge *h[N + 10];
	int post[N + 10];
	int nodeNum;
	void fix(int fixID){
		while (fixID > 1){
			int fat = fixID / 2;
			if (h[fixID]->weight< h[fat]->weight){
				swap(post[h[fixID]->head], post[h[fat]->head]);
				swap(h[fixID], h[fat]);
				fixID /= 2;
			}
			else
				break;
		}
	}
public:
Heap(){
		nodeNum = 0;
	}
	void push(int vertID, int w){
		nodeNum++;
		h[nodeNum] = (Edge *)malloc(sizeof(Edge));
		h[nodeNum]->head = vertID;
		h[nodeNum]->weight = w;
		post[vertID] = nodeNum;
		fix(nodeNum);
	}
	void update(int vertID, int w){
		int p = post[vertID];
		h[p]->weight = w;
		fix(p);
	}
	int pop(){
		int ret = h[1]->head;
		free(h[1]);
		h[1] = h[nodeNum--];
		if (nodeNum > 0)
			post[h[1]->head] = 1;

		int cur = 1;
		while (cur * 2 <= nodeNum){
			int son = cur * 2;
			if (son + 1 <= nodeNum && h[son]->weight > h[son + 1]->weight)
				son = son + 1;
			if (h[cur]->weight > h[son]->weight){
				swap(post[h[cur]->head], post[h[son]->head]);
				swap(h[cur], h[son]);
				cur = son;
			}
			else
				break;
		}

		return ret;
	}
	int empty(){
		if (nodeNum <= 0)
			return 1;
		else
			return 0;
	}
	void display(){
		printf("********************\n");
		for (int i = 1; i <= nodeNum; i++)
			printf("%d %d\n", h[i]->head, h[i]->weight);
		for (int i = 0; i < N; i++)
			printf("pot_%d: %d\n", i, post[i]);
		printf("********************\n");
	}
	~Heap(){
		while (nodeNum > 0)
			pop();
	}
};
void dijkstraWithHeap(Graph *G, int s, int t,int *DirtyMap){
    //printf("in dj\n");
	for (int i = 0; i < G->n; i++)
		if (i == s)
			d[i] = 0;
		else
			d[i] = INF;
	for (int i = 0; i < G->n; i++)
	{
		flag[i] = 0;
		pre[i] = -1;
		peg[i] = -1;
	}
	 // printf("in dj2\n");
	int cur = s;
	Heap heap;
	for (int i = 0; i < G->n; i++)
		heap.push(i, d[i]);
	//printf("iafter heap\n");
	do{
		 //printf("in while\n");
		int cur = heap.pop();
		flag[cur] = 1;
		if (cur == t)
			break;
		for (int i = 0; i<G->near[cur].size(); i++){
			//printf("in for id is %d\n",i);
			int id = G->near[cur][i];
			Edge* e = &(G->incL[id]);
			int delt = 1;
			if (DirtyMap[i]>=0 && d[e->head]>(d[e->tail] + e->weight) && flag[e->head] == 0){
			
				d[e->head] = d[e->tail] + e->weight;
				heap.update(e->tail, d[e->tail]);
				pre[e->head] = e->tail;
				peg[e->head] = id;
			}
		  // printf("out of if \n");
		}
	} while (!heap.empty());
}*/
void Compare::bellmanFord(Graph *G, int st, int te, int* p,int* nullmap){
	//printf("int...\n");
	float *d = new float[G->n*sizeof(float)];
	for (int i = 0; i < G->n; i++)
	{
		p[i] = -1;
		d[i] = 100000;
	}
	d[st] = 0;
	int change;
	int round = 0;
	do{
		//printf("here 1\n");
		change = 0;
		for (int i = 0; i < G->m; i++){
			Edge e = G->incL[i];
			//printf("%d\n", i);
			if (nullmap[i] >= 0&&(d[e.head] > d[e.tail] + e.weight))//&&e.weight==1)
			{
				change = 1;
				d[e.head] = d[e.tail] + e.weight;
				p[e.head] = i;
			}
		}
		round++;
	} while (change>0);
	//printf("here!\n");

}
int Compare::cmp(service s1, service s2)
{
	if (s1.d> s2.d)
		return 1;
	else
		return 0;
}
int Compare::Rough(Graph* G, vector<service> s)
{   
	float weight=0;
	int nullmap[60000];
	sort(s.begin(), s.end(), this->cmp);
	int pegg[10010];
	float capacity[10010];
	for (int f = 0; f < G->m; f++)
	{
		capacity[f] = G->incL[f].capacity;
		lambda[f] = 0;
	}
	int i = 0;
	int cnt = 0;
	float flow = 0;
	for (int k = 0; k < G->m; k++)
		nullmap[k] =1;
	for (i = 0; i < s.size(); i++)
	{
		//dijkstraWithHeap(G, s[i].s, s[i].t, nullmap);
		for (int j = 0; j < G->m; j++)
			if (capacity[j] <s[i].d)
			{
				//printf("in here!\n");
				nullmap[j] = -1;
			}
		bellmanFord(G, s[i].s, s[i].t,pegg,nullmap);
		//dijcapacity(G, s[i].s, s[i].t, dt, pegg, lambda, capacity,0);
		int f = pegg[s[i].t];
		//printf("f is %d\n",f);
		//printf("%d  %d\n", s[i].s, s[i].t);
		int hop = 0;
		while (f >= 0)
		{
			hop++;
			//printf("%d->", f);
			capacity[f] =capacity[f]-s[i].d;
			if (G->incL[f].tail == s[i].s)
			{
				cnt++;
				flow += s[i].d;
			}
			weight += G->incL[f].weight;
			f = pegg[G->incL[f].tail];
		}
		//printf("hop is :%d\n",hop);
	}

	return cnt;


}
