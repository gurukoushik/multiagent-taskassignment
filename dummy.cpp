// Column-wise assign
for(int i = 0; i < n; i++)
{
	for(int j = 0; j < n; j++)
	{
		if(cost_matrix[i][j] == 0 && goalassigned[j] == 0 && agentassigned[i] == 0 && numofcolzeros(i) == 1)
		{
			goalassigned[j] = 1;
			agentassigned[i] = 1;
		}
	}
}

// Tick unassigned rows
for(int i = 0; i < n; i++)
	if(agentassigned[i] == 0)
		tickedrows.push_back(i);

bool done = false;

while done == false
{
	for(int rowtick : tickedrows)
	{
		for(int j = 0; j < n; j++)
		{
			if (cost_matrix[rowtick][j] == 0)
				tickedcols.push_back(j);
		}
	}
	
}

// // Print agent assigned
// cout << "Row Cover" << endl;
// for(int i = 0; i < n; i++)
// 	cout << rowcover[i] << " ";
// cout << endl;
// // Print goal assigned
// cout << "Column Cover" << endl;
// for(int i = 0; i < n; i++)
// 	cout << colcover[i] << " ";
// cout << endl;


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
	vector<vector<int>> mask;
	vector<int> rowcover;
	vector<int> colcover;
	int saved_row;
	int saved_col;
public:
	taskassignment(vector<vector<float>> cost_matrix, int n)
	{
		this->cost_matrix = cost_matrix;
		this->n = n;
		vector<vector<int>> mask(n,vector<int> (n,0));
		vector<int> rowcover(n,0);
		vector<int> colcover(n,0);
		this->mask = mask; 
		this->rowcover = rowcover;
		this->colcover = colcover;
		saved_row = -1;
		saved_col = -1;
	}
	
	void rowminsubtract()
	{
		// Row minimum subtraction
		vector<float> rowmin(n, std::numeric_limits<float>::infinity());
		for(int i = 0; i < n; i++)
			rowmin[i] = *min_element(cost_matrix[i].begin(), cost_matrix[i].end());

		for(int i = 0; i < n; i++)
			for(int j = 0; j < n; j++)
				cost_matrix[i][j] = cost_matrix[i][j] - rowmin[i];
	}

	void colminsubtract()
	{
		// Column Minimum Subtraction
		vector<float> colmin(n, std::numeric_limits<float>::infinity());
		for(int j = 0; j < n; j++)
			for(int i = 0; i < n; i++)
				if (colmin[j] > cost_matrix[i][j]) 
					colmin[j] = cost_matrix[i][j];

		for(int j = 0; j < n; j++)
			for(int i = 0; i < n; i++)
				cost_matrix[i][j] = cost_matrix[i][j] - colmin[j];

	}

	int numofrowzeros(int row_index)
	{
		// Row zeros count
		int count = 0;
		
		for(int j = 0; j < n; j++)
			if (cost_matrix[row_index][j] == 0) 
				count++;

		return count;
	}

	int numofcolzeros(int col_index)
	{
		// Row zeros count
		int count = 0;
		
		for(int i = 0; i < n; i++)
			if (cost_matrix[i][col_index] == 0) 
				count++;

		return count;
	}

	void starzeros()
	{

		// Row-wise star zeros
		for(int i = 0; i < n; i++)
		{
			for(int j = 0; j < n; j++)
			{
				if(cost_matrix[i][j] == 0 && rowcover[j] == 0 && colcover[i] == 0)
				{
					mask[i][j] = 1;
					rowcover[j] = 1;
					colcover[i] = 1;
				}
			}
		}

		for(int i = 0; i < n; i++)
			rowcover[i] = 0;
		for(int i = 0; i < n; i++)
			colcover[i] = 0;

		print_matrix(mask);

	}

	bool coverstarcols()
	{
		bool done = false;
		// Row-wise star zeros
		for(int i = 0; i < n; i++)
			for(int j = 0; j < n; j++)
				if(mask[i][j] == 1)
					colcover[j] = 1;
		
		int columncount = 0;
		for(int j = 0; j < n; j++)
			if (colcover[j] == 1)
				columncount++;
		if(columncount >= n)
			done = true;

		return done;
	}


	bool starinrow(int row)
	{
		bool found =  false;
		for(int j = 0; j < n; j++)
			if(mask[row][j] == 1)
				found = true;
		return found;
	}

	void findstarinrow(int row, int &col)
	{
		col = -1;
		for(int j = 0; j < n; j++)
			if(mask[row][j] == 1)
				col = j;
	}

	void findzero(int &row, int &col)
	{
		int r = 0;
		int c;
		bool done = false;
		row = -1;
		col = -1;

		// while(!done)
		// {
		// 	c = 0;
		// 	while(true)
		// 	{
		// 		if(cost_matrix[r][c] == 0 && rowcover[r] == 0 && colcover[c] == 0)
		// 		{
		// 			row = r;
		// 			col = c;
		// 			done = true;
		// 		}
		// 		c++;
		// 		if (c >= n || done)
		// 			break;
		// 	}
		// 	r++;
		// 	if(r >= n)
		// 		done = true;
		// }
		for(int i = 0; i < n; i++)
		{
			if (!done)
			{
				for(int j = 0; j < n; j++)
				{
					if(cost_matrix[i][j] == 0 && rowcover[i] == 0 && colcover[j] == 0)
					{
						row = i;
						col = j;
						done = true;
					}
				}
			}
		}
	}

	int primeuncoveredzeros()
	{
		int row = -1;
		int col = -1;
		bool done = false;
		int step = 0;
		while(!done)
		{
			findzero(row,col);
			if(row == -1)
			{
				done = true;
				step = 6;
			}
			else
			{
				mask[row][col] = 2;
				if(starinrow(row))
				{
					findstarinrow(row,col);
					rowcover[row] = 1;
					colcover[col] = 0;
				}
				else
				{
					done = true;
					step = 5;
					saved_row = row;
					saved_col = col;
				}
			}
		}

		return step;
	}

	void findstarincol(int &row, int col)
	{
		row = -1;
		for(int i = 0; i < n; i++)
			if(mask[i][col] == 1)
				row = i;
	}

	void findprimeinrow(int row, int &col)
	{
		for(int j = 0; j < n; j++)
			if(mask[row][j] == 2)
				col = j;
	}

	void clearcovers()
	{
		for(int j = 0; j < n; j++)
			rowcover[j] = 0;
		for(int j = 0; j < n; j++)
			colcover[j] = 0;
	}

	void clearprimes()
	{
		for(int i = 0; i < n; i++)
			for(int j = 0; j < n; j++)
				if(mask[i][j] == 2)
					mask[i][j] = 0;
	}

	void print_matrix(vector<vector<auto>> matrix)
	{
		// Print matrix
		for(int i = 0; i < n; i++)
		{
			for(int j = 0; j < n; j++)
			{
				cout << matrix[i][j] << " ";
			}
			cout << endl;
		}
	}

	void hungarian()
	{
		bool done;
		rowminsubtract(); // Step 1
		starzeros(); // Step 2
		done = coverstarcols(); // Step 3
		if(done)
			cout << "Assignment Done" << endl;
		else
		{
			int step = primeuncoveredzeros();
			if(step == 5)
			{
				cout << "Comes to step 5" << endl;
			}
			else if (step == 6)
			{
				cout << "Comes to step 6" << endl;	
			}
			else
			{
				cout << "Soomething wrong in step 4" << endl;
			}
		}
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