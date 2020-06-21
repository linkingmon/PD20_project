#include "maze_router.h"

using namespace std;

// void Shortest_Path:

void Shortest_Path::InitializeSingleSource(){
  
    Set_boundary();
    grid_point_pointer.resize( width , vector<Coordinate*>(height));
    AdjList.resize( width , vector<list<pair<SP_direction,double> > > (height));
    visited.resize( width , vector<bool> (height,false));
    int z = 0;
    for(int x = left_bound ; x < right_bound ; x++){
        for(int y = bottom_bound ; y < top_bound ; y++){
            double distance = 100;
            Coordinate* temp = new Coordinate(x,y,z,distance);
            grid_point_Min_Heap.push_back( (*temp) );
            grid_point_pointer[x][y] = ( (temp ) );
        }
    }

    Construct_Edge();
}

void Shortest_Path::Set_boundary(){
    height = abs(target_x-source_x)* expand_factor;
    width =  abs(target_y-source_y)* expand_factor;

    left_bound = 0;
    bottom_bound = 0;
    height = 3;
    width = 3;
    right_bound = left_bound + width;
    top_bound = bottom_bound + height;


}

void Shortest_Path::Construct_Edge(){
    // for(size_t x = 0 ; x < width-1 ; x++){
    //     for(size_t y = 0 ; y < height-1 ; y++){
    //         for(int d = p_x ;  d != n_x ; d++){
    //             AdjList[x][y].push_back()
    //         }
    //     }
    // }
    Construct_H_Edge();
    Construct_V_Edge();
}

void Shortest_Path::Construct_H_Edge(){         // Construct horizontal edge
    cout<<"AdjList size is "<<AdjList.size()<<endl;
    cout<<"width is "<<width<<endl;
    cout<<"heifht is "<<height<<endl;
    int z = 0;
    for(size_t x = 0 ; x < width-1 ; x++){
        for(size_t y = 0 ; y < height ; y++){
            AdjList[x][y].push_back( pair<SP_direction,double>  (p_x , (*row_map)(x+left_bound, y+bottom_bound, z) ) );
            AdjList[x+1][y].push_back( pair<SP_direction,double> (n_x , (*row_map)(x+left_bound, y+bottom_bound, z) ) );
        }
        
    }
}

void Shortest_Path::Construct_V_Edge(){         // Construct horizontal edge

    int z = 0;
    for(size_t x = 0 ; x < width ; x++){
        for(size_t y = 0 ; y < height-1 ; y++){
            AdjList[x][y].push_back( pair<SP_direction,double> (p_y , (*col_map)(x+left_bound, y+bottom_bound, z) ) );
            AdjList[x][y+1].push_back( pair<SP_direction,double> (n_y , (*col_map)(x+left_bound, y+bottom_bound, z) ) );
        }
        
    }
}


void Shortest_Path::Dijkstra(){


    visited.resize( width , vector<bool>  (height,false));  // initializa visited[] as {0,0,0,...,0}
    MinHeap<Coordinate> min_Heap(grid_point_Min_Heap);
    min_Heap.print_heap();
    
    // priority_queue <Coordinate,vector<Coordinate>,Coordinate_Compartor> min_Heap(grid_point.begin(),grid_point.end());
    
    // cout<<"min heap' size is "<<min_Heap.size()<<endl;
    // Coordinate point_min = min_Heap.top();
    // cout<<c_min.distance<<endl;
    while (!min_Heap.empty()) {
        Coordinate point_min = min_Heap.ExtractMin();
        // int p1_id = point_min.id(width,height);
        // cout << p1_id<<endl;
        // return;
        int p1_x = point_min.x - left_bound;
        int p1_y = point_min.y - bottom_bound;
        cout << "p1 x is "<<p1_x<<" p1 y is "<<p1_y<<endl; 
        visited[p1_x][p1_y] = true;
        for (list<pair<SP_direction,double>>::iterator iter = AdjList[p1_x][p1_y].begin();
            iter != AdjList[p1_x][p1_y].end(); iter++) {
            SP_direction dir = (*iter).first;
            int p2_x = p1_x;
            int p2_y = p1_y;
            switch(dir){
                case(p_x):{
                    ++p2_x;
                    break;
                }
                case(n_x):{
                    --p2_x;
                    break;
                }
                case(p_y):{
                    ++p2_y;
                    break;
                }
                case(n_y):{
                    --p2_y;
                    break;
                }
            }
            cout << "p2 x is "<<p2_x<<" p2 y is "<<p2_y<<endl; 
            if(visited[p2_y][p2_y] == true) continue;

            Relax( grid_point_pointer[p1_x][p1_y], grid_point_pointer[p2_y][p2_y], (*iter).second);
            min_Heap.Decrease_key( (*grid_point_pointer[p2_y][p2_y]));
        }
        // return;
    }
    // std::cout << "\nprint predecessor:\n";
    // PrintDataArray(predecessor);
    // std::cout << "\nprint distance:\n";
    // PrintDataArray(distance);
}

void Shortest_Path::Relax( Coordinate* from, Coordinate* to, double weight ){
    
    if (to->distance > from->distance + weight) {
        to->distance = from->distance + weight;
        to->predecessor = from;
    }
}