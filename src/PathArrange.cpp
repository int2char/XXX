#include"Graph.h"
#include<vector>
#include<time.h>
#include <algorithm>
#include<fstream>
#include"BFS.h"
#include"stdio.h"
#include"dijkstra.h"
#include <iostream>
#include"routemask.h"
#include"const.h"
#include"math.h"
#include<fstream>
#include<map>
#include<iomanip>
#include"service.h"
#define SQ .5
#define	RO 80//90
#define MU 0//10
#define BON 1//0,1
#define N 10000
float dist[N];
int peg[N];
float bestflow;
float cbbflow;
vector<float> canswer;
void writejsondanswer(vector<pair<float,int>>&data,string method)
{
	ofstream midfile(ANSWERS,ios::app);
	midfile<<"{"<<"\"type\":"<<"\""<<TYPE<<"\""<<",";
	midfile<<"\"graph_type\":"<<"\""<<GRAPHTYPE<<"\""<<",";
	midfile<<"\"node\":"<<"\""<<NODE<<"\""<<",";
	midfile<<"\"edge\":"<<"\""<<EDge<<"\""<<",";
	midfile<<"\"task\":"<<"\""<<Task<<"\""<<",";
	midfile<<"\"capacity\":"<<"\""<<CAPACITY<<"\""<<",";
	midfile<<"\"method\":"<<"\""<<method<<"\""<<",";
	midfile<<"\"flows\":"<<"\"";
	for(int i=0;i<data.size();i++)
		midfile<<data[i].first<<" ";
	midfile<<"\""<<",";
	midfile<<"\"hops\":"<<"\"";
	for(int i=0;i<data.size();i++)
		midfile<<data[i].second<<" ";
	midfile<<"\"";
	midfile<<"}"<<endl;
	midfile.close();
}
bool cmpv(RouteMark a1, RouteMark a2){
	if (a1.length >a2.length)
		return true;
	return false;
}
float  rearrange(Graph* G, float *capacity, float *lambda, int*pre, float*d, float *pd, int *te, int *st, int num, int mum, double& bestadd, int&stillS, int wide, int len, vector<vector<int>>&StoreRoute, vector<vector<int>>&BestRoute,vector<vector<int>>&TmpRoute,vector<set<int> >&stpair, ostream& Out, vector<RouteMark>& bestroutes, double totalflow,vector<vector<int>>&mind)
{
	//float stt=float(1000*clock())/ CLOCKS_PER_SEC;
	totalflow=0;
	vector<RouteMark> Routes;
	vector<vector<int>>haslink(mum,vector<int>());
	for (int i = 0; i < mum; i++)
		capacity[i] = G->incL[i].capacity;
	vector<int> remain;
	for (int i = 0; i < num; i++)
	{
		int n = 0;			
		int f = pre[te[i] * wide + st[i]*len];
		if (StoreRoute[i][0] < 0)
		{
			while (f >= 0){
				n++;
				f = pre[G->incL[f].tail*wide + st[i]*len];
				if (n>1000)
					cout << "circle"<<i<< endl;
			}
		}
		else
			n = StoreRoute[i][0];
		float value = pow(pd[i], SQ) / ((float)n);
		Routes.push_back(RouteMark(value, i));
	}
	sort(Routes.begin(), Routes.end(), cmpv);
	//float as=float(1000*clock())/ CLOCKS_PER_SEC;
	//cout<<"after as "<<as-stt<<endl;
	for (int ai = 0; ai <Routes.size(); ai++)
	{
		int i = Routes[ai].mark;
		float demand = pd[i];
		int f = pre[te[i] * wide + st[i]*len];
		int n = 0;
		int flag = 0;
		if (StoreRoute[i][0] < 0)
		{
			StoreRoute[i].clear();
			StoreRoute[i].push_back(-2);
			while (f >= 0)
			{
				StoreRoute[i].push_back(f);
				haslink[f].push_back(i);
				if (capacity[f] < demand)
				{
					flag = 1;
					//break;
				}
				if (n > 1000)
				{
					printf("circle!!!in s:%d:\n", i);
				}
				f = pre[G->incL[f].tail*wide + st[i]*len];
				n++;
			}
			StoreRoute[i].push_back(-1);
			if(n>=INFHOPS)
				flag=1;
			if (flag == 0)
			{
				StoreRoute[i][0]=n;
				int j = 0;
				totalflow += demand*n;
				int f = pre[te[i] * wide + st[i]*len];
				while (f >= 0)
				{
					j++;
					capacity[f]-=demand;
					//haslink[f].push_back(i);
					f = pre[G->incL[f].tail*wide + st[i]*len];
					if (n > 10100)
						printf("erro2\n");
				}
			}
			else
			{
				totalflow+=demand*INFHOPS;
				remain.push_back(i);
			}
		}
		else
		{
			int j = 0;
			while (true)
			{
				j++;
				int edn = StoreRoute[i][j];
				if (edn < 0)
					break;
				haslink[edn].push_back(i);
				if (capacity[edn] < demand)
				{
					flag = 1;
					//break;
				}
			}
			if(j-1>=INFHOPS)
				flag=1;
			if (flag == 0)
			{
				j = 0;
				while (true)
				{
					j++;
					int edn = StoreRoute[i][j];
					if (edn < 0)
						break;
					//haslink[edn].push_back(i);
					capacity[edn] -= demand;
				}
				totalflow += demand*(j-1);
			}
			else
			{
				StoreRoute[i][0]=-2;
				totalflow+=demand*INFHOPS;
				remain.push_back(i);
			}
		}
	}
	float aadd=float(1000*clock())/ CLOCKS_PER_SEC;
	//cout<<"after add "<<aadd-as<<endl;
	float *bca =new float[mum];
	for(int i=0;i<mum;i++)
		bca[i]=capacity[i];
	TmpRoute=StoreRoute;
	if (BON>0)
	{
		int ren = remain.size();
		vector<int> stillremain;
		for (int i = 0; i < ren; i++){
			float demand = pd[remain[i]];
			int hops=StoreRoute[remain[i]].size()-2;
			BFSO(G, st[remain[i]], te[remain[i]], dist, peg,demand,bca,hops,mind);
			int f = peg[te[remain[i]]];
			if (dist[te[remain[i]]]<INFHOPS)
			{
				TmpRoute[remain[i]].clear();
				TmpRoute[remain[i]].push_back(dist[te[remain[i]]]);
				int j = 0;
				while (f >= 0)
				{

					if (bca[f] < demand)
						printf("erro!\n");
					bca[f] -= demand;
					TmpRoute[remain[i]].push_back(f);
					haslink[f].push_back(remain[i]);
					f = peg[G->incL[f].tail];
					j++;
				}
				TmpRoute[remain[i]].push_back(-1);
				totalflow += demand*(j-INFHOPS);
			}
			else
			{
				stillremain.push_back(remain[i]);
				TmpRoute[remain[i]].clear();
				TmpRoute[remain[i]].push_back(-1);
			}

		}
	}
	float tflow = totalflow;
	if (tflow<bestadd)
	{
		cout<<"( "<<tflow<<" )"<<endl;
		bestadd = tflow;
		cbbflow = bestadd;
		bestroutes = Routes;
		for (int i = 0; i < num; i++)
		{
			BestRoute[i].clear();
			int j = 0;
			while (true)
			{
				BestRoute[i].push_back(TmpRoute[i][j]);
				if (TmpRoute[i][j] < 0)
					break;
				j++;
			}
		}
	}
	else
		cout<<tflow<<endl;
	float au=float(1000*clock())/ CLOCKS_PER_SEC;
	//cout<<"after bfs "<<au-aadd<<endl;
	stillS = remain.size();
	int maskC = 0;
	set<int>remainset;
	set<int>added;
	for (int i = 0; i < stillS; i++)
	{
		float demand = pd[remain[i]];
		int j = 0;
		int cc=0;
		vector<int>vedge;
		while (true)
		{
			j++;
			int edn = StoreRoute[remain[i]][j];
			if (edn < 0)
				break;
			if (demand>capacity[edn])
				{
					vedge.push_back(edn);
					stpair[st[remain[i]]].insert(te[remain[i]]);
					maskC++;
				}
		}
		int gg=rand()%vedge.size();
		if(added.find(vedge[gg])==added.end())
			{
				lambda[vedge[gg]]+=1;
				added.insert(vedge[gg]);
			}
		for(int k=0;k<vedge.size();k++)
		{
			int ran=rand()%90;
			if(ran>RO)
			{
				if(added.find(vedge[k])==added.end())
				{
					lambda[vedge[k]]+=1;
					added.insert(vedge[k]);
				}
			}
		}
	}
	//float am=float(1000*clock())/ CLOCKS_PER_SEC;
	//cout<<"after lamnda add "<<am-au<<endl;
	stillS = maskC;
	return tflow;
}
/*float  rearrange(Graph* G, float *capacity, float *lambda, int*pre, float*d, float *pd, int *te, int *st, int num, int mum, double& bestadd, int&stillS, int wide, int len, vector<vector<int>>&StoreRoute, vector<vector<int>>&BestRoute,vector<vector<int>>&TmpRoute,vector<set<int> >&stpair, ostream& Out, vector<RouteMark>& bestroutes, double totalflow)
{
	totalflow=0;
	vector<RouteMark> Routes;
	vector<vector<int>>haslink(mum,vector<int>());
	for (int i = 0; i < mum; i++)
	{
		capacity[i] = G->incL[i].capacity;
	}
	vector<int> remain;
	for (int i = 0; i < num; i++)
	{
		int n = 0;			
		int f = pre[te[i] * wide + st[i]*len];
		if (StoreRoute[i][0] < 0)
		{
			while (f >= 0){
				n++;
				f = pre[G->incL[f].tail*wide + st[i]*len];
				if (n>1000)
					cout << "circle"<<i<< endl;
			}
		}
		else
			n = StoreRoute[i][0];
		float value = pow(pd[i], SQ) / ((float)n);
		Routes.push_back(RouteMark(value, i));
	}
	sort(Routes.begin(), Routes.end(), cmpv);
	for (int ai = 0; ai <Routes.size(); ai++)
	{
		int i = Routes[ai].mark;
		float demand = pd[i];
		int f = pre[te[i] * wide + st[i]*len];
		int n = 0;
		int flag = 0;
		if (StoreRoute[i][0] < 0)
		{
			while (f >= 0)
			{
				haslink[f].push_back(i);
				if (capacity[f] < demand)
				{
					flag = 1;
					//break;
				}
				if (n > 1000)
				{
					printf("circle!!!in s:%d:\n", i);
				}
				f = pre[G->incL[f].tail*wide + st[i]*len];
				n++;
			}
			if(n>=INFHOPS)
				flag=1;
			if (flag == 0)
			{
				StoreRoute[i].clear();
				StoreRoute[i].push_back(n);
				int j = 0;
				totalflow += demand*n;
				int f = pre[te[i] * wide + st[i]*len];
				while (f >= 0)
				{
					j++;
					StoreRoute[i].push_back(f);
					capacity[f]-=demand;
					haslink[f].push_back(i);
					f = pre[G->incL[f].tail*wide + st[i]*len];
					if (n > 10100)
						printf("erro2\n");
				}
				StoreRoute[i].push_back(-1);
			}
			else
			{
				totalflow+=demand*INFHOPS;
				remain.push_back(i);
			}
		}
		else
		{
			int j = 0;
			while (true)
			{
				j++;
				int edn = StoreRoute[i][j];
				if (edn < 0)
					break;
				haslink[edn].push_back(i);
				if (capacity[edn] < demand)
				{
					flag = 1;
					//break;
				}
			}
			if(j-1>=INFHOPS)
				flag=1;
			if (flag == 0)
			{
				j = 0;
				while (true)
				{
					j++;
					int edn = StoreRoute[i][j];
					if (edn < 0)
						break;
					haslink[edn].push_back(i);
					capacity[edn] -= demand;
				}
				totalflow += demand*(j-1);
			}
			else
			{
				totalflow+=demand*INFHOPS;
				remain.push_back(i);
			}
		}

	}
	time_t trs = clock();
	stillS = remain.size();
	int maskC = 0;
	set<int>remainset;
	for(int i=0;i<remain.size();i++)
	{
		//maskC++;
		remainset.insert(remain[i]);
		stpair[st[remain[i]]].insert(te[remain[i]]);
	}
	for (int i = 0; i < stillS; i++)
	{
		if(StoreRoute[remain[i]][0]<0)
		{
			float demand = pd[remain[i]];
			int f = pre[te[remain[i]] * wide + st[remain[i]] * len];
			int max = 0;
			int mm = -1;
			int cc=0;
			while (f >= 0)
			{
				vector<int>vedge;
				if (demand>capacity[f]);
				{
					vedge.push_back(f);
					int r = rand() % 90;
					if (r>RO)
						{
							lambda[f] += 1;
							stpair[st[remain[i]]].insert(te[remain[i]]);
							StoreRoute[remain[i]].clear();
							StoreRoute[remain[i]].push_back(-1);
							//maskC++;
							for(int i=0;i<haslink[f].size();i++)
								{
									if(remainset.find(haslink[f][i])!=remainset.end())
									{
										stpair[st[haslink[f][i]]].insert(te[haslink[f][i]]);
										maskC++;
									}
								}
						}
				}
				//cout<<"cc is: "<<cc<<endl;
				f = pre[G->incL[f].tail*wide + st[remain[i]] * len];
			}
		}
		else
		{
			float demand = pd[remain[i]];
			int j = 0;
			int cc=0;
			vector<int>vedge;
			while (true)
			{
				j++;
				int edn = StoreRoute[remain[i]][j];
				vedge.push_back(edn);
				if (edn < 0)
					break;
				if (demand>capacity[edn]);
				{
					//cc++;
					int r = rand() % 90;
					if (r>RO)
						{
							lambda[edn] += 1;
							stpair[st[remain[i]]].insert(te[remain[i]]);
							StoreRoute[remain[i]].clear();
							StoreRoute[remain[i]].push_back(-1);
							//maskC++;
							for(int i=0;i<haslink[edn].size();i++)
								{
									stpair[st[haslink[edn][i]]].insert(te[haslink[edn][i]]);
									//maskC++;
									for(int i=0;i<haslink[edn].size();i++)
										{
											if(remainset.find(haslink[edn][i])!=remainset.end())
											{
												stpair[st[haslink[edn][i]]].insert(te[haslink[edn][i]]);
												maskC++;
											}
										}
								}
						}
				}
				//cout<<"cc is "<<cc<<endl;
			}
		}
	}
	TmpRoute=StoreRoute;
	if (BON>0)
	{
		int ren = remain.size();
		vector<int> stillremain;
		for (int i = 0; i < ren; i++){
			float demand = pd[remain[i]];
			BFS(G, st[remain[i]], te[remain[i]], dist, peg, demand, capacity);
			int f = peg[te[remain[i]]];
			if (dist[te[remain[i]]]<INFHOPS)
			{
				TmpRoute[remain[i]].clear();
				TmpRoute[remain[i]].push_back(dist[te[remain[i]]]);
				int j = 0;
				while (f >= 0)
				{

					if (capacity[f] < demand)
						printf("erro!\n");
					capacity[f] -= demand;
					TmpRoute[remain[i]].push_back(f);
					haslink[f].push_back(remain[i]);
					f = peg[G->incL[f].tail];
					j++;
				}
				TmpRoute[remain[i]].push_back(-1);
				totalflow += demand*(j-INFHOPS);
			}
			else
			{
				stillremain.push_back(remain[i]);
				TmpRoute[remain[i]].clear();
				TmpRoute[remain[i]].push_back(-1);
			}

		}
		stillS = stillremain.size();
	}
	float tflow = totalflow;
	if (tflow<bestadd)
	{
		cout<<"( "<<tflow<<" )"<<endl;
		bestadd = tflow;
		cbbflow = bestadd;
		bestroutes = Routes;
		for (int i = 0; i < num; i++)
		{
			BestRoute[i].clear();
			int j = 0;
			while (true)
			{
				BestRoute[i].push_back(TmpRoute[i][j]);
				if (TmpRoute[i][j] < 0)
					break;
				j++;
			}
		}
	}
	else
		cout<<tflow<<endl;
	stillS = maskC;
	cout<<"maskC is "<<maskC<<endl;
	return tflow;
}*/
float morein(Graph* G, float capacity[], float pd[], int te[], int st[], int num, int mum,vector<vector<int>>&StoreRoute,ostream& Out, vector<RouteMark>& bestroutes){
	float totalflow = 0;
	int max = 100*Task*INFHOPS;
	for (int i = 0; i < canswer.size(); i++)
	{
		if (i % 10 == 0)
			Out << endl;
		if (canswer[i]<max)
			Out << "(";
		Out << canswer[i];
		if (canswer[i]<max)
		{
			Out << ")";
			max = canswer[i];
		}
		Out << " ";
	}
	if (BON == 0){
		for (int i = 0; i < mum; i++)
		{
			capacity[i] = G->incL[i].capacity;
		}
		vector<int>remain;
		for (int ai = 0; ai <bestroutes.size(); ai++)
		{
			int i = bestroutes[ai].mark;
			float demand = pd[i];
			int n = 0;
			if (StoreRoute[i][0]>0)
			{
				
				int j = 0;
				while (true)
				{
					j++;
					int edn = StoreRoute[i][j];
					if (edn < 0)
						break;
					capacity[edn] -= demand;
				}
				totalflow += demand*j;

			}
			else
				remain.push_back(i);
		}

		{
			int ren = remain.size();
			for (int i = 0; i < ren; i++){
				float demand = pd[remain[i]];
				BFS(G, st[remain[i]], te[remain[i]], dist, peg, demand, capacity,10000000);
				int f = peg[te[remain[i]]];
				int ran = rand() % 10;
				if (dist[te[remain[i]]] <10)
				{
					StoreRoute[remain[i]][0] = dist[te[remain[i]]];
					int j = 0;
					while (f >= 0)
					{
						j++;
						if (capacity[f] < demand)
							printf("erro!\n");
						capacity[f] -= demand;
						StoreRoute[remain[i]][j] = f;
						f = peg[G->incL[f].tail];
					}
					StoreRoute[remain[i]][++j] = -1;
					totalflow += demand*j;
				}
				else
				{
					StoreRoute[remain[i]][0] = -1;
					totalflow+=demand*INFHOPS;
				}

			}
		}
		Out <<endl<< "esmate gap is:" << (totalflow-cbbflow) << endl;
		Out << "esmate result is:" << endl;
		max = 100 * Task*INFHOPS;
		for (int i = 0; i < canswer.size(); i++)
		{
			if (i % 10 == 0)
				Out << endl;
			if (canswer[i] < max)
				Out << "(";
			Out << canswer[i] + (totalflow-cbbflow);
			if (canswer[i] <max)
			{
				Out << ")";
				max = canswer[i];
			}
			Out << " ";

		}
	}
	return totalflow;
}
vector<pair<int, vector<int> > > GetResult(Graph* G, int st[], int te[], float pd[], int pre[], int num, int mum, int wide, int len)
{
	vector<pair<int, vector<int> > > result;
	float capacity[10020];
	for (int i = 0; i < mum; i++)
	{
		capacity[i] = G->incL[i].capacity;
	}
	vector<int> remain;
	//printf("n:%d m:%d", num,mum);
	for (int i = 0; i <num; i++)
	{
		float demand = pd[i];
		//printf("%f\n", demand);
		int f = pre[te[i] * wide + i*len];
		int n = 0;
		int flag = 0;

		while (f >= 0)
		{
			n++;
			if (capacity[f] < demand)
			{
				flag = 10;
				break;
			}
			if (n > 1000)
			{
				printf("circle!!!in s:%d:\n", i);

			}
			//printf("%d<-", f);

			f = pre[G->incL[f].tail*wide + i*len];

		}
		if (flag == 0)
		{
			vector<int>temp;
			int n = 0;
			int f = pre[te[i] * wide + i*len];
			while (f >= 0)
			{
				capacity[f] -= demand;
				temp.push_back(f);
				f = pre[G->incL[f].tail*wide + i*len];
				n++;
				if (n > 10100)
					printf("erro2\n");
			}
			//printf("\n");
			result.push_back(make_pair(i, temp));
		}
		else
			remain.push_back(i);

	}
	//printf("remain is %d\n", remain.size());
	int ren = remain.size();

	vector<int> stillremain;
	for (int i = 0; i < ren; i++){
		float demand = pd[remain[i]];
		BFS(G, st[remain[i]], te[remain[i]], dist, peg, demand, capacity,100000000);
		//dijcapacity(G, st[remain[i]], te[remain[i]], d, peg, lambda, capacity, demand);
		int f = peg[te[remain[i]]];
		vector<int> temp;
		while (f >= 0)
		{
			capacity[f] -= demand;
			temp.push_back(f);
			f = peg[G->incL[f].tail];
		}
		if (temp.size()>0)
			result.push_back(make_pair(remain[i], temp));
	}
	return result;
}
vector<pair<int, vector<int>>> GrabResult(vector<vector<int>>&Route, int taskn, int ednum, float* pd){
	vector<pair<int, vector<int> > > result;
	int addin = 0;
	float *capacity = (float*)calloc(ednum, sizeof(float));
	float tweight=0;
	for (int i = 0; i < taskn; i++)
	{
		if (Route[i][0] < 0)
			{	
				tweight+=pd[i]*INFHOPS;
				continue;
			}
		int j = 0;
		addin++;
		vector<int>temp;
		while (true)
		{
			j++;
			if (Route[i][j] < 0)
				break;
			temp.push_back(Route[i][j]);
		}
		tweight+=pd[i]*(j-1);
		result.push_back(make_pair(i, temp));
		temp.clear();
	}
	cout<<"tweight is: "<<tweight<<endl;

	return result;
}
pair<float,int> CheckR(Graph*G, vector<pair<int, vector<int>> > result,vector<service>&ser,string method)
{
	cout<<"in check"<<endl;
	float lowbound=0;
	for(int i=0;i<Task;i++)
		lowbound+=ser[i].d*INFHOPS;
    float addinpart=0;
	float totalflow=0;
	float checkobject=0;
	float* capacity=new float[G->m];
	int mum = G->m;
	vector<pair<float,int>> answers;
	for (int i = 0; i <mum; i++)
	{
		capacity[i] =G->incL[i].capacity;
	}
	float totalweight = 0;
	float objective=0;
	for (int i = 0; i < result.size(); i++)
	{

		int flag = 0;
		//int flag2 = 0;
		float demand = ser[result[i].first].d;
		totalflow += demand;
		if (G->incL[result[i].second[0]].head != ser[result[i].first].t)
		{
			cout<<"erro inresult:wrong route/unreach temination/service"<<i<<endl;
			//return;
		}
		//printf("s:%d,t:%d\n", st[result[i].first], te[result[i].first]);
		for (int j = 0; j < result[i].second.size(); j++)
		{
			totalweight += 1;// G->incL[result[i].second[j]].weight;
			capacity[result[i].second[j]] -= demand;
			//printf("%d<-", G->incL[result[i].second[j]].head, G->incL[result[i].second[j]].tail);
			if (capacity[result[i].second[j]] < 0)
			{
				cout << "erro in result!!!:overload edge" << result[i].second[j] << " " << result[i].first << "edge cap :" << capacity[result[i].second[j]]<<"demand"<<demand<<endl;
				//return;
			}
			if (G->incL[result[i].second[j]].tail != ser[result[i].first].s)
				if (G->incL[result[i].second[j]].tail != G->incL[result[i].second[j + 1]].head)
				{
						cout<<"erro inresult:wrong route/unconected route/service"<<result[i].first<<endl;
					//return make_pair(-1,-1);
				}
			if (G->incL[result[i].second[j]].tail == ser[result[i].first].s)
			{
				flag = 1;
			}
		}
		int len=result[i].second.size();
		addinpart+=(len-INFHOPS)*demand;
		//Out<<demand<<" "<<len<<endl;
		//printf("\n");
		if (flag == 0)
		{
				cout<<"erro in result: wrong route/start wrong/service:"<<result[i].first<<endl;
			return make_pair(-1,-1);
		}
		answers.push_back(make_pair(demand,len));
	}
	 for(int i=0;i<result.size();i++)
                        {
			reverse(result[i].second.begin(),result[i].second.end());
                        map<int,int>remap;
                        int first=0;
                        for(int j=0;j<result[i].second.size();j++)
                                {if(first==0)
                                        {
                                        remap.insert(make_pair(G->incL[result[i].second[j]].tail,1));
                                        first=1;
                                        }
                                if(remap.find(G->incL[result[i].second[j]].head)!=remap.end())
                                        cout<<"erro loop here"<<endl;
                                remap.insert(make_pair(G->incL[result[i].second[j]].head,1));
                                }
			remap.clear();
                        }
                        

	cout<<"check corrected!!!"<<endl;
   // cout<<"total weight is:"<<totalweight<<endl;
	//cout<<"total flow is:"<<totalflow<<endl;
	//cout<< "total addin is:" << result.size()<<endl;	
	cout<<"lowbound is:"<<lowbound<<endl;
	//cout<<"not addin is:"<<totalflow*INFHOPS<<endl;
	cout<<"addinpart is:"<<addinpart<<endl;
	writejsondanswer(answers,method);
	return make_pair(totalflow,totalweight) ;


}
void CheckRoute(int**Route, int taskn, int ednum, float* pd){
	float *capacity = (float*)calloc(ednum,sizeof(float));
	float tflow = 0;
	int pathnum = 0;
	for (int i = 0; i < taskn; i++)
		{
		if (Route[i][0] < 0)
				continue;
		int j = 0;
		tflow += pd[i];
		//pathnum+=
		while (true)
		{ 
			j++;
			if (Route[i][j] < 0)
				break;
			capacity[Route[i][j]] +=pd[i];
			if (capacity[Route[i][j]]>100)
			{
				cout << "erro ovr capacity!" << endl;
				return;
			}
		}
	}
	cout << "right no erro" << endl<<"totol flow check is:"<<tflow<<endl;
	delete[]capacity;

}
void writejsoniter(char*filename,vector<float>&iter,string method)
{               
                cout<<setprecision(12);
		ofstream midfile(filename,ios::app);
		midfile<<"{"<<"\"type\":"<<"\""<<TYPE<<"\""<<",";
		midfile<<"\"graph_type\":"<<"\""<<GRAPHTYPE<<"\""<<",";
		midfile<<"\"node\":"<<"\""<<NODE<<"\""<<",";
		midfile<<"\"edge\":"<<"\""<<EDge<<"\""<<",";
		midfile<<"\"task\":"<<"\""<<Task<<"\""<<",";
		midfile<<"\"capacity\":"<<"\""<<CAPACITY<<"\""<<",";
		midfile<<"\"method\":"<<"\""<<method<<"\""<<",";
		midfile<<"\"iter\":"<<"\"";
		for(int i=0;i<iter.size();i++)
			midfile<<std::fixed<<iter[i]<<" ";
		midfile<<"\""<<"}"<<endl;
		midfile.close();

}
void writejsondata(char*filename,vector<pair<string,float>>&data,string method)
{
	ofstream midfile(filename,ios::app);
	midfile<<"{"<<"\"type\":"<<"\""<<TYPE<<"\""<<",";
	midfile<<"\"graph_type\":"<<"\""<<GRAPHTYPE<<"\""<<",";
	midfile<<"\"node\":"<<"\""<<NODE<<"\""<<",";
	midfile<<"\"edge\":"<<"\""<<EDge<<"\""<<",";
	midfile<<"\"task\":"<<"\""<<Task<<"\""<<",";
	midfile<<"\"capacity\":"<<"\""<<CAPACITY<<"\""<<",";
	midfile<<"\"method\":"<<"\""<<method<<"\""<<",";
	cout<<setprecision(12);
	float obj=data[0].second;
	float inf_obj=data[1].second;
	//float test=123456578910;
	midfile<<"\"assess\":"<<"\""<<float(obj/inf_obj)<<"\"";
	//data.push_back(make_pair("test:",test));
	for(int i=0;i<data.size();i++)
		{
		midfile<<",";
		midfile<<"\""<<data[i].first<<"\":"<<"\""<<std::fixed<<data[i].second<<"\"";
		}
	midfile<<"}"<<endl;
	midfile.close();
}
