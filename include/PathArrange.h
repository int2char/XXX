#include"Graph.h"
#include<vector>
#include<ostream>
#include"routemask.h"
#include<fstream>
#include"service.h"
#include<set>
using namespace std;
inline float  rearrange(Graph* G, float *capacity, float *lambda, int*pre, float*d, float *pd, int *te, int *st, int num, int mum, double& bestadd, int&stillS, int wide, int len, vector<vector<int>>&StoreRoute, vector<vector<int>>&BestRoute,vector<vector<int>>&TmpRoute,vector<set<int> >&stpair, ostream& Out, vector<RouteMark>& bestroutes, double totalflow,vector<vector<int>>&);
inline float rearrange2(Graph* G, float *capacity, float *lambda, int*pre, float*d, float *pd, int *te, int *st, int num, int mum, double& bestadd, int&stillS, int wide, int len, vector<vector<int>>&StoreRoute, vector<vector<int>>&BestRoute,vector<vector<int>>&TmpRoute,int* mask, ostream& Out, vector<RouteMark>& bestroutes, double totalflow);
vector<pair<int, vector<int> > > GetResult(Graph* G, int st[], int te[], float pd[], int pre[], int num, int mum, int wide, int len);
pair<float,int>CheckR(Graph*G, vector<pair<int, vector<int>> > result,vector<service> &ser,string method);
void CheckRoute(int**Route, int taskn, int ednum, float*pd);
vector<pair<int, vector<int>>> GrabResult(vector<vector<int>>&Routes, int taskn, int ednum, float* pd);
float inline morein(Graph* G, float capacity[], float pd[], int te[], int st[], int num, int mum,vector<vector<int>>&, ostream& Out, vector<RouteMark>& bestroutes);
void writejsoniter(char*filename,vector<float>&iter,string method);
void writejsondata(char*filename,vector<pair<string,float>>&iter,string method);
void writejsondanswer(vector<pair<float,int>>&data,string method);
