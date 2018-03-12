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
	int biao = head*NODE + i;
	float val = c[tail*NODE + i]+1 +lambda[tid];
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
	int biao = head*NODE + i;
	float val = c[tail*NODE + i]+1+lambda[tid];
	if (c[biao] == val){
		p[biao] = tid;
	}
}
__global__ void ChangePameterC(int*p, float*d, int* st){
	int tid = blockIdx.y;
	int i = threadIdx.x + blockDim.x*blockIdx.x;
	if (i >= NODE || tid >= NODE)return;
	int biao = tid*NODE + i;
	d[biao] = (i == tid) ? 0.0 : 10000000000.0;
	p[biao] = -1;
}
void GraphPath::Copy2GPU(std::vector<service> &s){
	for (int i = 0; i < Task; i++)
	{
		st[i] = s[i].s;
		te[i] = s[i].t;
		pd[i] = (float)s[i].d;
	}
	vector<int>vecmask;
	for (int i = 0; i < Task; i++)
		vecmask.push_back(st[i]);
	sort(vecmask.begin(),vecmask.end());
	vector<int>::iterator begin=vecmask.begin();
	vector<int>::iterator end=unique(vecmask.begin(),vecmask.end());
	stillS=0;
	for(begin;begin<end;begin++)
		mask[stillS++]=*begin;
	for (int i = 0; i < EDge; i++)
		lambda[i] = 0;
	cudaMemcpy(dev_st, st, Task*sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpy(dev_te, te, Task*sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpy(dev_lambda, lambda, EDge*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(dev_mask, mask, Task*sizeof(int), cudaMemcpyHostToDevice);
}
GraphPath::GraphPath(Graph&_G,vector<vector<int>>&_mind):G(_G),StoreRoute(Task, vector<int>(1,-1)), BestRoute(Task, vector<int>()),mind(_mind)
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
	st = new int[Task];
	te = new int[Task];
	pd = new float[Task];
	d = new float[NODE*NODE];
	pre = new int[NODE*NODE];
	lambda = new float[EDge];
	mask = new int[NODE];
	mark = new int(1);
	capacity = (float*)malloc(EDge*sizeof(float));
	for (int i = 0; i < NODE; i++)
		{
			for (int j = 0; j <NODE; j++)
			{
				if (j == i)
				{
					d[i*NODE+i] = 0.0;
					pre[i*NODE+i] = -1;
				}
				else
				{
					d[i*NODE +j] = 100000.0;
					pre[i*NODE+j] = -1;
				}
			}
		}
	cudaMemcpy(dev_d, d, NODE*NODE*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(dev_p, pre, NODE*NODE*sizeof(int), cudaMemcpyHostToDevice);
}
bool scmp(service s1,service s2)
{
	if(s1.s<s2.s)
		return true;
	return false;
}
vector<pair<string,float> > GraphPath::bellmanFordCuda(vector<service>&ser,ostream& Out) {
	printf("Lagrange parrel searching..............\n");
	srand(time(NULL));
	float start = float(1000*clock())/ CLOCKS_PER_SEC;
	sort(ser.begin(),ser.end(),scmp);
	Copy2GPU(ser);
	int num = Task;
	int mum = EDge;
	int reme = 0;
	int count = 0;
	vector<RouteMark> bestroutes;
	devicesize += 2 * Task*sizeof(RouteMark);
	int bestround = 0;
	int zeor = 0;
	double totalflow = 0;
	for (int i = 0; i < Task; i++)
		totalflow += INFHOPS *pd[i];
	double bestadd = totalflow;
	float best = totalflow;
	vector<float>middata;
	vector<vector<int>>TmpRoute(Task, vector<int>(1,-1));
	vector<set<int> >stset(NODE,set<int>());
	for(int i=0;i<num;i++)
		stset[st[i]].insert(te[i]);
	for (int i = 0; i <10000000; i++)
	{
		float ss=float(1000*clock())/ CLOCKS_PER_SEC;
		count++;
		reme++;
		stillS=0;
		for(int k=0;k<NODE;k++)
			if(!stset[k].empty())
				mask[stillS++]=k;
		dim3 blocksq(stillS/threadsize + 1, NODE*stillS/stillS);
		ChangePameterC << <blocksq, threadsize >> >(dev_p, dev_d, dev_st);
		cudaMemcpy(dev_lambda, lambda, EDge*sizeof(float), cudaMemcpyHostToDevice);
		cudaMemcpy(dev_mask, mask, stillS*sizeof(int), cudaMemcpyHostToDevice);
		dim3 blocks_square(stillS / threadsize + 1, EDge*stillS /stillS);
		do{
			cudaMemcpy(dev_m, &zeor, sizeof(int), cudaMemcpyHostToDevice);
			bellmanHigh << <blocks_square, threadsize >> >(dev_edge, dev_m, dev_d, dev_p, dev_lambda, dev_mask, stillS);
			cudaMemcpy(mark, dev_m, sizeof(int), cudaMemcpyDeviceToHost);
		} while (*mark);
		color << <blocks_square, threadsize >> >(dev_edge, dev_m, dev_d, dev_p, dev_lambda, dev_mask, stillS);
		cudaMemcpy(pre, dev_p, sizeof(int)*NODE*NODE, cudaMemcpyDeviceToHost);
		cudaMemcpy(d, dev_d, sizeof(float)*NODE*NODE, cudaMemcpyDeviceToHost);
		float ee=float(1000*clock())/ CLOCKS_PER_SEC;
		//cout<<"gpu time is "<<ee-ss<<endl;
		int value = rearrange(&G, capacity, lambda, pre, d, pd, te, st, num, mum, bestadd, stillS, NODE, 1, StoreRoute, BestRoute,TmpRoute,stset,Out, bestroutes,totalflow,mind);
		float eee=float(1000*clock())/ CLOCKS_PER_SEC;
		//cout<<"rearrange time is "<<eee-ee<<endl;
		middata.push_back(value);
		if (value<best)
		{
			bestround = count;
			best = value;
			reme = 0;
		}
		if (stillS == 0 || reme>loomore)
			break;
	}
	float end=float(1000*clock())/ CLOCKS_PER_SEC;
	vector<pair<int, vector<int>>> result = GrabResult(BestRoute, num, mum, pd);
	int addin = result.size();
	pair<float,int> tf=CheckR(&G, result,ser,string("Lag_Parallel"));
	writejsoniter(LAGPFILE,middata,string("Lag_Parallel"));
	vector<pair<string,float>> rdata;
	rdata.push_back(make_pair(string("object"),best));
	rdata.push_back(make_pair(string("inf_obj"),totalflow));
	rdata.push_back(make_pair(string("task_add_in"),addin));
	rdata.push_back(make_pair(string("flow_add_in"),tf.first));
	rdata.push_back(make_pair(string("total_weight"),tf.second));
	rdata.push_back(make_pair(string("time"),(end-start)));
	rdata.push_back(make_pair(string("iter_num"),count));
	rdata.push_back(make_pair(string("iter_time"),float(end-start)/(float)count));
	writejsondata(DATAFILE,rdata,string("Lag_Parallel"));
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
	/*delete[] st;
	delete[] te;
	delete[] pd;
	delete[]d;
	delete[]pre;
	delete[] lambda;
	delete[] mask;
	delete mark;
	free(capacity);*/
}



