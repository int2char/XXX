#include"Graph.h"
#include"dijkstra.h"
#include"Heap.h"
#include<queue>
#include"math.h"
#define INF 100000
#define N 10000
#include<cstring>
#include<iostream>
using namespace std;
void BFS(Graph *G, int s, int t,float d[], int peg[], float demand, float capacity[],float rv){
        int n_num = G->n;
        memset(peg,-1,n_num*sizeof(int));
        for(int i=0;i<n_num;i++)
                d[i]=INF;
        d[s]=0;
        int cur = s;
        queue<int>que;
        que.push(s);
        do{
                cur=que.front();
                que.pop();
                if (cur==t||d[cur]>=2*rv)break;
                int size = G->near[cur].size();
                for (int i= size-1; i>=0; i--)
                {
                        int id = G->near[cur][i];
                        if (capacity[id]<demand)continue;
                        int head = G->incL[id].head;
                        int tail = G->incL[id].tail;
                        int w=d[tail]+1;
                        if (d[head]>w)
                        {
                           d[head]=w;
                           peg[head]=id;
                           que.push(head);
                        }
                }
        } while (!que.empty());
}     
void BFSO(Graph *G, int s, int t,float d[], int peg[], float demand, float capacity[],float rv,vector<vector<int>>&mind){
        int n_num = G->n;
        memset(peg,-1,n_num*sizeof(int));
        for(int i=0;i<n_num;i++)
                d[i]=INF;
        d[s]=0;
        int cur = s;
        queue<int>que;
        que.push(s);
        do{
                cur=que.front();
                que.pop();
                if(mind[cur][t]+d[cur]>2*rv)continue;
                if (cur==t||d[cur]>=2*rv)break;
                int size = G->near[cur].size();
                for (int i= size-1; i>=0; i--)
                {
                        int id = G->near[cur][i];
                        if (capacity[id]<demand)continue;
                        int head = G->incL[id].head;
                        int tail = G->incL[id].tail;
                        int w=d[tail]+1;
                        if (d[head]>w)
                        {
                           d[head]=w;
                           peg[head]=id;
                           que.push(head);
                        }
                }
        } while (!que.empty());
}     