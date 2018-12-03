#include<iostream>
#include<vector>
#include<queue>
#include <unordered_set>
using namespace std;

const int INF = 1000000007;

class Graph{
    private:
    vector<vector<int>> square_grid;  //world which is a square_grid consists of 0's and 1's, o-free space, 1- occupied space
    
    // right, down, left, up directions in which robot can move
   const int dx[4]={1,0,-1,0}; 
   const int dy[4] = {0,1,0,-1};
   
    
public:
    Graph(const vector<vector<int>> &g){
        square_grid=g;
    }
    int makeVertex(int x, int y) const {      // with x and y making the vertex
        return x *square_grid[0].size() + y;
    }
    int getRow(int v) const {                    //function to get the row of the square grid
        return v / square_grid[0].size();
    }
    int getColumn(int v) const {                 //function to get the column of the square grid
        return v % square_grid[0].size();
    }
    vector<int> getNeighbours(int v) const {      //function to get all the adjacent vertices for a given vertex and also checking whether those vertices are legal and within the bounds
        vector<int> neighbors;
        for (int i = 0; i < 4; ++i){
            int x = getRow(v) + dx[i], y = getColumn(v) + dy[i];
            if (x < 0 || x >= square_grid.size()) continue;
            if (y < 0 || y >= square_grid[0].size()) continue;
            if (square_grid[x][y]==1){
                continue;
            }
            neighbors.push_back(makeVertex(x,y));
        }
        return neighbors;
    }
    int size() const {
        return square_grid.size() * square_grid[0].size();
    }
    
};

int heuristic(const Graph& graph, int v, int w) {                 //heuristic function for calculating the expected cost for the robot to move from current vertex to goal.
    int v_row = graph.getRow(v), v_column = graph.getColumn(v);
    int w_row = graph.getRow(w), w_column = graph.getColumn(w);
    return abs(v_row-w_row) + abs(v_column-w_column);
}

void Astar(const Graph& graph, int start, int end) {             //Astar algorithm
    // Dijkstra's algorithm
  vector<int> shortest; // vector<vector<int>> shortest;
  vector<int> prev; // vector<vector<int>> prev;
  shortest.resize(graph.size()+5);
  prev.resize(graph.size()+5);
  priority_queue<pair<int,int>, vector<pair<int,int>>, greater<pair<int,int>>> q; // priorit_queue<pair<int,pair<int,int>, ...> q;
  unordered_set<int> closed_set; // unordered_set<pair<int,int> closed_set;
  for (int i = 1; i <= graph.size(); ++i)
    shortest[i] = INF;
  shortest[start] = 0;
  prev[start] = -1;
  q.push(make_pair(0,start));
  while(!q.empty()) {
    pair<int,int> proposition = q.top();
    q.pop();
    int v = proposition.second, pathCost = shortest[v];
    cout << "Current vertex: " << v << "\n";
    if (closed_set.count(v) > 0){
       continue;
    }
    closed_set.insert(v);
    if (v == end) break;
    vector<int> neighbours = graph.getNeighbours(v);
    for (int item : neighbours) {
      int neighbour = item, edgeCost = 1;
      int newPath = pathCost + edgeCost;
      int estimation = newPath + heuristic(graph,neighbour,end);
      if (shortest[neighbour] > newPath) {
        shortest[neighbour] = newPath;
        prev[neighbour] = v;
        q.push(make_pair(estimation, neighbour));
      }
    }
  }
  
  cout << "Shortest path cost: " << shortest[end] << "\n";
  // Reconstruct the path
  vector<int> path;
  int curr_vertex = end;
  while(curr_vertex != -1) {
    path.push_back(curr_vertex);
    curr_vertex = prev[curr_vertex];
  }
  cout << "Path: ";
  for (int i = path.size()-1; i >= 0; --i)
    cout  << "(" << graph.getRow(path[i]) << "," << graph.getColumn(path[i]) << ") ";
    // cout << path[i] << " ";
  cout << "\n";
}

int main() {
  vector<vector<int>>square_grid = {{0, 0, 1, 0, 0, 0},

        {0, 0, 1, 0, 0, 0},

        {0, 0, 0, 0, 1, 0},

        {0, 0, 1, 1, 1, 0},

        {0, 0, 0, 0, 1, 0},

{0, 0, 0, 0, 0, 0}};
Graph g(square_grid);
int start=g.makeVertex(2,0);
int end=g.makeVertex((square_grid.size()-1),(square_grid[0].size()-1));
Astar(g,start,end);
return 0;
}
