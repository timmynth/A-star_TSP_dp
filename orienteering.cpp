#include <iostream>
#include <vector>
#include <cstdio>
#include <queue>
#include <cmath>
#include <algorithm>

using namespace std;

//#define debug_mode

int dir1[] = {1,0,-1,0};
int dir2[] = {0,1,0,-1};


struct node {
    int cpx, cpy;
};

struct point {
    char symbol;
    bool reachable;
    int x, y;
    float f,g,h;//for a star search
};

struct minheap_comparator { //comparator for minheap
    bool operator()(point a, point b) {
        return  a.f > b.f;
    }
};

class Orienteering
{
public:
    int w,h; //
    int num_node;						//number of node reachable (node = G S @)
    int num_cp; 						//number of checkpoints reachable
    int num_cp_real; 					//number of checkpoints
    int Sx, Sy, Gx, Gy;					//start and goal loactions
    point ** board; 					//2D board of point
    node nd[20]; 						//array of node, at most 20 (18+2)
    int adj_matrix[20][20]; 			//adjacency matrix for nodes
    vector< vector< int > > graph, dp;	//for tsp

    void main();						//main
    void init_var();					//init the var
    int init_board();					//create the board and read the points
    void DFS_reachable(int,int); 		//Find all reachable points from S
    float heuristic(int,int,int,int);	//Euclidean distance
    int A_start_2_pt(int,int); 			//return the shortest path lengths of the two points by A star
    void update_adj_matrix(); 			//fill in the adjacency matrix

    void free_board();					//free the board memory
    void print_debug();					//for debug messages
    int TSP(int,int);					//recursive TSP
    int TSP_wrapper();					//return the TSP solution
};

void Orienteering::main()
{
    //TODO: Implement this function
    init_var();

    if (1 != init_board()) {
        cout << -1;
        return;
    }

    DFS_reachable(Sy,Sx);

    if (!board[Gy][Gx].reachable) {
        cout << -1;
        return;
    }

    update_adj_matrix();

    cout << TSP_wrapper();
    free_board();
}

void Orienteering::init_var()
{
    num_node=0;
    num_cp = 0;
    num_cp_real = 0;
    Sx = Sy = Gx = Gy = -1;
}

int Orienteering::init_board()
{

    cin >> w >> h;
    if (w<=0 || w>100 || h <=0 || h>100 ) {
        return 0;
    }
    //create board
    board = new point* [h];
    for (int i = 0; i< h; i++) {
        board[i] = new point[w];
    }
    for (int i = 0; i< h; i++) {
        for (int j = 0 ; j< w; j++) {
            cin >> board[i][j].symbol;
            board[i][j].reachable = false;
            board[i][j].x= j;
            board[i][j].y= i;
            if (board[i][j].symbol == 'S') {
                if (Sx != -1 && Sy != -1)
                    return 0;
                Sx = j;
                Sy = i;
            } else if (board[i][j].symbol == 'G') {
                if (Gx != -1 && Gy != -1)
                    return 0;
                Gx = j;
                Gy = i;
            } else if (board[i][j].symbol == '@') {
                num_cp_real++;
                if (num_cp_real>18)
                    return 0;
            }
        }
    }
	
	if (Sx == -1 || Sy == -1 || Gx == -1 || Gy ==-1)
        return 0;

    return 1;
}

void Orienteering::DFS_reachable(int y, int x)
{
    board[y][x].reachable = true;
    int pos;
    switch (board[y][x].symbol) {
    case 'S':
        pos = 0;
        break;
    case 'G':
        pos = 1;
        break;
    case '@':
        pos = 2 + num_cp;
        num_cp++;
        break;
    default: ;
    }
    if (board[y][x].symbol != '.') {
        num_node++;
        nd[pos].cpx = x;
        nd[pos].cpy = y;
    }
    for (int i = 0; i<4; i++) {
        int yy = y+dir2[i];
        int xx = x+dir1[i];
        if ((yy<h) && (yy>=0) && (xx<w) && (xx >=0) && (board[yy][xx].symbol!='#') && (board[yy][xx].reachable == false)) {
            DFS_reachable(yy,xx);
        }
    }
}

float Orienteering::heuristic(int x1, int y1, int x2, int y2)
{
    return sqrt((float)pow(abs(x1-x2),2) + pow(abs(y1-y2),2));
}

int Orienteering::A_start_2_pt(int n1,int n2)
{
    bool visited[100][100];
    bool expanded[100][100];
    priority_queue<point, std::vector<point>, minheap_comparator> minheap;
    for (int i = 0; i< w; i++) {
        for (int j = 0; j<h; j++) {
            visited[j][i] = false;
            expanded[j][i] = false;
            board[j][i].g =1000;
            board[j][i].f =0;
            board[j][i].h =0;
        }
    }
    int cx = nd[n1].cpx; //start x,y
    int cy = nd[n1].cpy;
    int gx = nd[n2].cpx; //goal x, y
    int gy = nd[n2].cpy;

    board[cy][cx].g = 0;
    board[cy][cx].h = heuristic(cx,cy,gx,gy);
    board[cy][cx].f = board[cy][cx].h;
    expanded[cy][cx] = true;
    minheap.push(board[cy][cx]);
    int heap_count = 1;

    while(heap_count != 0) {

	label:
        point current = minheap.top();
        int nowx = current.x;
        int nowy = current.y;

        //check if the poped is up-to-date, otherwise pop it and top another
        if (board[nowy][nowx].g < current.g) {
            minheap.pop();
            goto label;
        }

        //it is really the new one to pop, goal-test it then pop (since immutable min heap in C++)
        //goal test
        if ((nowy == gy) && (nowx == gx))
            return ((int)board[nowy][nowx].g);
        minheap.pop();
        expanded[nowy][nowx] = false;
        heap_count--;

#ifdef debug_mode
        cout << nowy << " " << nowx << " " << board[nowy][nowx].g << " "<< board[nowy][nowx].h << " "<< board[nowy][nowx].f << " out heap" << endl;
#endif

        //include current point to visited
        visited[nowy][nowx] = true;

        for (int i = 0; i<4; i++) {
            int nexty = nowy+dir1[i];
            int nextx = nowx+dir2[i];

            if ((nexty<h) && (nexty >=0) && (nextx<w) && (nextx>=0) && (board[nexty][nextx].reachable)) {
                //a valid neighbour point

                if (visited[nexty][nextx])
                    continue;

                float g = board[nowy][nowx].g + 1;

                if (!expanded[nexty][nextx]) {
                    board[nexty][nextx].g = g;
                    board[nexty][nextx].h = heuristic(nextx, nexty, gx, gy);
                    board[nexty][nextx].f= g+board[nexty][nextx].h;
                    minheap.push(board[nexty][nextx]);
                    expanded[nexty][nextx] = true;
                    heap_count++;
#ifdef debug_mode
                    cout << nexty << " " << nextx << " " <<  board[nexty][nextx].g<< " " <<  board[nexty][nextx].h << " " << board[nexty][nextx].f << " in heap" << endl;
#endif
                } else { //expanded already
                    if (g < board[nexty][nextx].g) {
                        //need to reorder the min-heap
                        board[nexty][nextx].g = g;
                        board[nexty][nextx].f= g+board[nexty][nextx].h;
                        minheap.push(board[nexty][nextx]);
                    }
                }


            }
        }
    }
    return -1;
}

void Orienteering::update_adj_matrix()
{
    for (int i = 0; i< num_node; i++) {
        for (int j =i; j< num_node; j++) {
            if (i==j) {
                adj_matrix[j][i] = 0;
                adj_matrix[i][j] = 0;
            } else {
                int distance = A_start_2_pt(i,j);
                adj_matrix[j][i] = distance;
                adj_matrix[i][j] = distance;
            }
        }
    }
}

int Orienteering::TSP(int status, int x)
{
    if ( dp[status][x] != -1 )
        return dp[status][x];

    int mask = 1<<x;
    dp[status][x] = 1e9;
    for (int i = 0; i < num_node; i++ )
        if ( i != x && ( status & ( 1 << i ) ) )
            dp[status][x] = min( dp[status][x], TSP(status-mask, i) + graph[i][x] );
    return dp[status][x];

}

int Orienteering::TSP_wrapper()
{
    int src = 0;
    int goal = 1;

    graph = vector< vector< int > >( num_node, vector< int >( num_node ) );
    dp = vector< vector< int > >( 1 << num_node, vector< int >( num_node, -1 ) );

    for (int i = 0; i < num_node; i++ )
        for (int j = 0; j < num_node; j++ ) {
            graph[i][j] =  adj_matrix[i][j];
        }

    //init
    for ( int i = 0; i < num_node; i++ )
        dp[1 << i][i] = graph[goal][i];

    return TSP( (1 << num_node) - 1, src);
}

void Orienteering::free_board()
{
    for (int i =0; i<h; i++) {
        delete [] board[i];
    }
    delete [] board;
}

void Orienteering::print_debug()
{
    cout <<"===========debug===========" << endl;
    cout << "W = " << w << " H = " << h << endl << endl;
    for (int i = 0; i< h; i++) {
        for (int j = 0 ; j< w; j++) {
            cout << board[i][j].symbol;
        }
        cout << endl;
    }

    cout << endl << "From the nd array view" << endl;
    cout << "There are " << num_node << " reachable nodes, they are " << endl;
    for (int i =0; i< num_node; i++) {
        cout << "Number " << i << ": " << nd[i].cpy << " " << nd[i].cpx  << endl;
    }
    cout << endl;

    cout << endl << "Reachability: " << endl;
    for (int i = 0; i< h; i++) {
        for (int j = 0 ; j< w; j++) {
            cout << board[i][j].reachable;
        }
        cout << endl;
    }
    cout << endl;

    cout << "Adj matrix" << endl;
    for (int j =0; j< num_node; j++) {
        for (int i =0; i<  num_node; i++) {
            cout << adj_matrix[j][i] << "\t";
        }
        cout << endl;
    }
    cout << endl;

}

int main()
{
    Orienteering o;
    o.main();
    return 0;
}
