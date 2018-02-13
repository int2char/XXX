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
float  rearrange(Graph* G, float *capacity, float *lambda, int*pre, float*d, float *pd, int *te, int *st, int num, int mum, double& bestadd, int&stillS, int wide, int len, vector<vector<int>>&StoreRoute, vector<vector<int>>&BestRoute, int* mask, ostream& Out, vector<RouteMark>& bestroutes, double totalflow)
{
	totalflow=0;
	vector<RouteMark> Routes;
	for (int i = 0; i < mum; i++)
	{
		capacity[i] = G->incL[i].capacity;
	}
	vector<int> remain;
	for (int i = 0; i < num; i++)
	{
		int n = 0;			
		int f = pre[te[i] * wide + i*len];
		if(st[i]==419&&te[i]==98)
			cout<<f<<" ";
		if (StoreRoute[i][0] < 0)
		{
			while (f >= 0){
				n++;
				f = pre[G->incL[f].tail*wide + i*len];
				if(st[i]==419&&te[i]==98)
							cout<<f<<" ";
				if (n>1000)
					cout << "circle"<<i<< endl;
			}
		}
		else
			n = StoreRoute[i][0];
		float value = pow(pd[i], SQ) / ((float)n);
		Routes.push_back(RouteMark(value, i));
		//cout << i << " " << endl;
	}
	sort(Routes.begin(), Routes.end(), cmpv);
	for (int ai = 0; ai <Routes.size(); ai++)
	{
		int i = Routes[ai].mark;
		float demand = pd[i];
		int f = pre[te[i] * wide + i*len];
		int n = 0;
		int flag = 0;
		if (StoreRoute[i][0] < 0)
		{
			while (f >= 0)
			{
				if (capacity[f] < demand)
				{
					flag = 1;
					break;
				}
				if (n > 1000)
				{
					printf("circle!!!in s:%d:\n", i);
					//break;
				}
				f = pre[G->incL[f].tail*wide + i*len];
				n++;
			}
			//new add bounaries here.
			if(n>=INFHOPS)
				flag=1;
			if (flag == 0)
			{
				StoreRoute[i].clear();
				StoreRoute[i].push_back(n);
				int j = 0;
				totalflow += demand*n;
				int f = pre[te[i] * wide + i*len];
				while (f >= 0)
				{
					j++;
					StoreRoute[i].push_back(f);
					capacity[f] -= demand;
					f = pre[G->incL[f].tail*wide + i*len];
					if (n > 10100)
						printf("erro2\n");
				}
				StoreRoute[i].push_back(-1);
				//printf("\n");
			}
			else
			{
				totalflow+=demand*INFHOPS;
				remain.push_back(i);
				StoreRoute[i].clear();
				StoreRoute[i].push_back(-1);
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
				if (capacity[edn] < demand)
				{
					flag = 1;
					break;
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
					capacity[edn] -= demand;
				}
				totalflow += demand*(j-1);
			}
			else
			{
				StoreRoute[i].clear();
				StoreRoute[i].push_back(-1);
				totalflow+=demand*INFHOPS;
				remain.push_back(i);
			}
		}

	}
	time_t trs = clock();
	stillS = remain.size();
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
						StoreRoute[remain[i]].clear();
						StoreRoute[remain[i]].push_back(dist[te[remain[i]]]);
						int j = 0;
						while (f >= 0)
						{

							if (capacity[f] < demand)
								printf("erro!\n");
							capacity[f] -= demand;
							StoreRoute[remain[i]].push_back(f);
							f = peg[G->incL[f].tail];
							j++;
						}
						StoreRoute[remain[i]].push_back(-1);
						totalflow += demand*(j-INFHOPS);
					}
					else
					{
						stillremain.push_back(remain[i]);
						StoreRoute[remain[i]].clear();
						StoreRoute[remain[i]].push_back(-1);
					}

				}
				stillS = stillremain.size();
			}
	float tflow = totalflow;
	if (tflow<bestadd)
	{
		bestadd = tflow;
		cbbflow = bestadd;
		bestroutes = Routes;
		for (int i = 0; i < num; i++)
		{
			BestRoute[i].clear();
			int j = 0;
			while (true)
			{
				BestRoute[i].push_back(StoreRoute[i][j]);
				if (StoreRoute[i][j] < 0)
					break;
				j++;
			}
		}
	}
	int maskC = 0;
	for (int i = 0; i < num; i++)
	{
		if (StoreRoute[i][0] < 0)
			mask[maskC++] = i;
		else
		{
			int random = rand() % 10;
			if (random < MU)
			{
				mask[maskC++] = i;
				StoreRoute[i].clear();
				StoreRoute[i].push_back(-1);
			}
		}
	}
	for (int i = 0; i < stillS; i++)
	{
		float demand = pd[remain[i]];
		int f = pre[te[remain[i]] * wide + remain[i] * len];
		int max = 0;
		int mm = -1;
		while (f >= 0)
		{

			if (demand>capacity[f]);
			{
				int r = rand() % 90;
				if (r>RO)
					lambda[f] += 1;
			}
			f = pre[G->incL[f].tail*wide + remain[i] * len];
		}
	}
	stillS = maskC;
	return tflow;
}
float morein(Graph* G, float capacity[], float pd[], int te[], int st[], int num, int mum,vector<vector<int>>&StoreRoute,ostream& Out, vector<RouteMark>& bestroutes){
	//Out << "node:" << G->n << "edge:" << G->m << "Task" << num << endl;
	float totalflow = 0;
	//Out << endl << "actual result is:" << endl;
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
				BFS(G, st[remain[i]], te[remain[i]], dist, peg, demand, capacity);
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
		BFS(G, st[remain[i]], te[remain[i]], dist, peg, demand, capacity);
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
	for (int i = 0; i < taskn; i++)
	{
		if (Route[i][0] < 0)
			continue;
		int j = 0;
		addin++;
		vector<int>temp;
		//cout << endl;
		//cout << i << "(" << Route[i][0] <<")"<< ":";
		while (true)
		{
			j++;
			if (Route[i][j] < 0)
				break;
			temp.push_back(Route[i][j]);
			//cout << Route[i][j] << "<-";
		}
		result.push_back(make_pair(i, temp));
		temp.clear();
	}
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
		addinpart+=len*demand;
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
    //cout<<"total weight is:"<<totalweight<<endl;
	//cout<<"total flow is:"<<totalflow<<endl;
	//cout<< "total addin is:" << result.size()<<endl;	//cout<<"lowbound is:"<<lowbound<<endl;
	//cout<<"not addin is:"<<totalflow*INFHOPS<<endl;
	//cout<<"addinpart is:"<<addinpart<<endl;
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
