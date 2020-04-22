#include <iostream>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <bits/stdc++.h>

using namespace std;

class taskassignment
{
private:
	vector<vector<float>> cost_matrix;
	int n;
public:
	taskassignment(vector<vector<float>> cost_matrix, int n)
	{
		this->cost_matrix = cost_matrix;
		this->n = n;
	}
	void rowminsubtract()
	{
		// Row minimum subtraction
		vector<float> rowmin(n, std::numeric_limits<float>::infinity());
		for(int i = 0; i < n; i++)
		{
			rowmin[i] = *min_element(cost_matrix[i].begin(), cost_matrix[i].end());
		}
		for(int i = 0; i < n; i++)
		{
			for(int j = 0; j < n; j++)
			{
				cost_matrix[i][j] = cost_matrix[i][j] - rowmin[i];
			}
		}
	}
	void colminsubtract()
	{
		// Column Minimum Subtraction
		vector<float> colmin(n, std::numeric_limits<float>::infinity());
		for(int j = 0; j < n; j++)
		{
			for(int i = 0; i < n; i++)
			{	
				if (colmin[j] > cost_matrix[i][j]) 
					colmin[j] = cost_matrix[i][j];
			}
		}
		for(int j = 0; j < n; j++)
		{
			for(int i = 0; i < n; i++)
			{
				cost_matrix[i][j] = cost_matrix[i][j] - colmin[j];
			}
		}

	}

	void print_matrix()
	{
		// Print matrix
		for(int i = 0; i < n; i++)
		{
			for(int j = 0; j < n; j++)
			{
				cout << cost_matrix[i][j] << " ";
			}
			cout << endl;
		}
	}

	void hungarian()
	{
		rowminsubtract();
		colminsubtract();
		print_matrix();
	}
};

int main()
{
	cout << "Hungarian Method for nxn optimal assignment\n\n";

	vector<vector<float>> cost_matrix = {{11,7,10,17,10},{13,21,7,11,13},{13,13,15,13,14},{18,10,13,16,14},{12,8,16,19,10}};

	taskassignment t1(cost_matrix,5);
	t1.hungarian();

	return 0;
}