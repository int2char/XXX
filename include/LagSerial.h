#ifndef __LagSerial__
#define __LagSerial__

#include "Graph.h"
#include"service.h"
#include"taskPath.h"
#include<iostream>
#include<ostream>
#include <vector>
using namespace std;
class LagSerial
{
public:
	vector<pair<string,float> > dijkstraSerial(vector<service> &s,ostream&Out = cout);
	LagSerial(Graph &_G);
	~LagSerial();
private:
	Graph G;
	int devicesize;
	int *st;
	int *te;
	float *pd;
	float *d;
	int *pre;
	float *lambda;
	int* mask;
	vector<vector<int>>StoreRoute;
	vector<vector<int>>BestRoute;
	float*capacity;
};

#endif
