#pragma once
#define Kb 30
#include<vector>
using namespace std;
class taskPath
{
public:
	int num;
	vector<int> Pathset[Kb + 2];
public:
	taskPath();
	~taskPath();
};

