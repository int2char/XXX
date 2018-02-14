#include "GraphPath.h"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
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
#include"BFS.h"
#include"taskPath.h"
#include"const.h"
#include"routemask.h"
#include"PathArrange.h"
#include<fstream>
using namespace std;
#define threadsize 256

bool UDgreater(pair<int, float> elem1, pair<int, float> elem2)
{
	return elem1.second > elem2.second;
}
bool UPGservice(service s1, service s2)
{
	return s1.d>s2.d;
}
bool cmp(float a, float b)
{
	return a<b;
}
__global__ void bellmanHigh(Edge *edge, int *m, float *c, int*p, float*lambda, int*mask, int stillS)
{
	int tid = blockIdx.y;
	int i = threadIdx.x + blockIdx.x*blockDim.x;
	if (i >= stillS)return;
	i = mask[i];
	int head = edge[tid].head;
	int tail = edge[tid].tail;
	int biao = head*Task + i;
	float val = c[tail*Task + i]+1 +lambda[tid];
	if (c[biao] >val){
		*m = 1;
		c[biao] = val;
	}
}
__global__ void color(Edge *edge, int *m, float *c, int*p, float*lambda, int *mask, int stillS){

	int tid = blockIdx.y;
	int i = threadIdx.x + blockIdx.x*blockDim.x;
	if (i >= stillS)return;
	i = mask[i];
	int head = edge[tid].head;
	int tail = edge[tid].tail;
	int biao = head*Task + i;
	float val = c[tail*Task + i]+1+lambda[tid];// * pd[i];
	if (c[biao] == val){
		p[biao] = tid;
	}
}
__global__ void ChangePameterC(int*p, float*d, int* st, int taskSize, int n){
	int tid = blockIdx.y;
	int i = threadIdx.x + blockDim.x*blockIdx.x;
	if (i >= taskSize || tid >= n)return;
	int biao = tid*taskSize + i;
	d[biao] = (st[i] == tid) ? 0.0 : 10000000000.0;
	p[biao] = -1;


}
void GraphPath::Copy2GPU(std::vector<service> &s){
	for (int i = 0; i < Task; i++)
	{
		st[i] = s[i].s;
		te[i] = s[i].t;
		pd[i] = (float)s[i].d;
	}
	for (int i = 0; i < Task; i++)
		mask[i] = i;
	for (int i = 0; i < EDge; i++)
		lambda[i] = 0;
	cudaMemcpy(dev_st, st, Task*sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpy(dev_te, te, Task*sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpy(dev_lambda, lambda, EDge*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(dev_mask, mask, Task*sizeof(int), cudaMemcpyHostToDevice);
}


GraphPath::GraphPath(Graph&_G):G(_G),StoreRoute(Task, vector<int>(1,-1)), BestRoute(Task, vector<int>())
{
	cudaMalloc(&dev_edge, sizeof(Edge)*EDge);
	cudaMemcpy(dev_edge, G.incL, EDge* sizeof(Edge), cudaMemcpyHostToDevice);
	cudaMalloc((void**)&dev_st, Task*sizeof(int));
	cudaMalloc((void**)&dev_te, Task*sizeof(int));
	cudaMalloc((void**)&dev_pd, Task*sizeof(float));
	cudaMalloc((void**)&dev_lambda, EDge*sizeof(float));
	cudaMalloc((void**)&dev_mask, Task*sizeof(int));
	cudaMalloc((void**)&dev_d, Task*NODE* sizeof(float));
	cudaMalloc((void**)&dev_p, Task*NODE* sizeof(int));
	cudaMalloc(&dev_m, sizeof(int));
	st = new int[Task*sizeof(int)];
	te = new int[Task*sizeof(int)];
	pd = new float[Task*sizeof(float)];
	d = (float*)malloc(Task*NODE*sizeof(float));
	pre = (int*)malloc(Task*NODE*sizeof(int));
	lambda = new float[EDge*sizeof(float)];
	mask = new int[Task];
	mark = new int(1);
	capacity = (float*)malloc(EDge*sizeof(float));
	for (int i = 0; i < NODE; i++)
		{
			for (int j = 0; j < Task; j++)
			{
				if (st[j] == i)
				{
					d[i*Task + j] = 0.0;
					pre[i*Task + j] = -1;
				}
				else
				{
					d[i*Task + j] = 100000.0;
					pre[i*Task + j] = -1;
				}
			}
		}
	cudaMemcpy(dev_d, d, Task*NODE*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(dev_p, pre, Task*NODE*sizeof(int), cudaMemcpyHostToDevice);
}
vector<pair<string,float> > GraphPath::bellmanFordCuda(vector<service>&ser,ostream& Out) {
	printf("Lagrange parrel searching..............\n");
	srand(time(NULL));
	vector<pair<string,float>> rdata;
	return rdata;
}
void GraphPath::CudaFree(){
	cudaFree(dev_st);
	cudaFree(dev_te);
	cudaFree(dev_pd);
	cudaFree(dev_lambda);
	cudaFree(dev_mask);
	cudaFree(dev_d);
	cudaFree(dev_p);
	cudaFree(dev_m);

}
GraphPath::~GraphPath()
{
	CudaFree();
	delete[] st;
	delete[] te;
	delete[] pd;
	free(d);
	free(pre);
	delete[] lambda;
	delete[] mask;
	delete mark;
	free(capacity);
}



