#include<iostream>
#include<ctime>
#include<vector>
#include<queue>
#include<climits>
#include<unordered_set>
using namespace std;
class Graph{
private:
    int start;//starting configuration of the holonomic robot
    int goal;//destination for the robot
    //robot has 2 degree of freedom and it is allowed to move in orthogonal moves
    int dx[4]={0,1,-1,0};
    int dy[4]={1,0,0,-1};
    
    //world which is a square_grid
    vector<vector<int>> square_grid;
    
public:
    Graph(const vector<vector<int>> &g){
        square_grid=g;
    }
    //initializing all the cells as vertices
    int makeVertex(int x,int y)const{
        return x*square_grid[0].size()+y;
    }
    //function to get the row number
    int getRow(int v)const{
        return v/square_grid.size();
        
    }
    //function to get the column number
    int getColumn(int v)const{
        return v % square_grid.size();
    }
    //function to get the total size of our square_grid
    int size()const{
        return square_grid.size()*square_grid[0].size();
    }
    //function to get all the adjacent vertices for each vertex and also checks whether it is legal or not and within the limits or not
    vector<int> getNeighbours(int v)const{
       vector<int> neighbors;
       for(int i=0;i<4;++i){
           int x=getRow(v);
           int y=getColumn(v);
           x += dx[i]; y += dy[i];
           if(x<0 || x >= square_grid.size()){
               continue;
           }
           if(y<0|| y>=square_grid.size()){
               continue;
           }
           if(square_grid[x][y]==1){
               continue;
           }
           neighbors.push_back(makeVertex(x,y));
       }
return neighbors; 
    }
};
//checking all the neighbors are visited
bool allVisited(const vector<int> &v,const unordered_set<int> &visited){
    for(int i=0;i<v.size();++i){
        if (visited.count(v[i])==0){
            return false;
        }
    }
    return true;
}
//Random Planner where the robot chooses the vertices randomly
void RandomPlanner(const Graph &g,int start,int goal,int max_step){
    unordered_set<int> visited;
    vector<int> randomMovements;
    randomMovements.push_back(start);
    visited.insert(start);
    int current_position=start;
    while(randomMovements.size()<max_step){
        vector<int> neighbors=g.getNeighbours(current_position);
        if (allVisited(neighbors,visited)) break;
        int r;
        do {
            r = rand() % neighbors.size();
            //cout << "random: " << r << "\n";
        }while(visited.count(neighbors[r]) > 0);
        current_position=neighbors[r];
        randomMovements.push_back(current_position);
        visited.insert(current_position);
        if(current_position==goal){
            break;
        }
    }
    if(current_position!=goal){
        cout<<"Random Planner failed"<<endl;
        return;
    }
    //cout<<randomMovements.size();
    //printing the path
    for(int i=0;i<randomMovements.size();++i){
        cout  << "(" << g.getRow(randomMovements[i]) << "," << g.getColumn(randomMovements[i]) << ") ";
    // cout << path[i] << " ";
  cout << "\n";
    }
}

int main(){
    srand(time(NULL));
    vector<vector<int>>square_grid = {{0, 0, 1, 0, 0, 0},
        {0, 0, 1, 0, 0, 0},

        {0, 0, 0, 0, 1, 0},

        {0, 0, 1, 1, 1, 0},

        {0, 0, 0, 0, 1, 0},

         {0, 0, 0, 0, 0, 0}};
Graph g(square_grid);
int start=g.makeVertex(2,0);
int goal=g.makeVertex((square_grid.size()-1),(square_grid[0].size()-1));
int max_step=9;//maximum step size
RandomPlanner(g,start,goal,max_step);
return 0;
}

