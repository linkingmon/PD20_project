#include "maze_router.h"
#include <assert.h>

using namespace std;

// void Shortest_Path:

void Shortest_Path::InitializeSingleSource(){
  
    Set_boundary();
    grid_point_pointer.resize( width , vector<Coordinate*>(height));
    AdjList.resize( width , vector<list<pair<SP_direction,double> > > (height));
    visited.resize( width , vector<bool> (height,false));
    int z = 0;
    for(int x = 0 ; x < width ; x++){
        for(int y = 0 ; y < height ; y++){
            double distance = 100;
            Coordinate* temp = new Coordinate(x+left_bound, y+bottom_bound, z, distance);
            temp->estimate_distance = Estimate_distance_to_target(x+left_bound ,y+bottom_bound);
            grid_point_Min_Heap.push_back( (*temp) );
            grid_point_pointer[x][y] = ( (temp ) );
        }
    }

    Construct_Edge();
}

int Shortest_Path::Estimate_distance_to_target(int x, int y){
    // x and y is original coordinate
    return abs(target_x - x) + abs(target_y - y );
}

void Shortest_Path::Set_boundary(){
    height = abs(target_x-source_x)* expand_factor;
    width =  abs(target_y-source_y)* expand_factor;

    left_bound = 0;
    bottom_bound = 0;
    height = 5;
    width = 5;
    right_bound = left_bound + width;
    top_bound = bottom_bound + height;

    
    left_bound = source_x < target_x ? source_x : target_x;
    bottom_bound = source_y < target_y ? source_y : target_y;
    right_bound = source_x > target_x ? source_x : target_x;;
    top_bound = source_y > target_y ? source_y : target_y;
    height = top_bound - bottom_bound + 1;
    width = right_bound - left_bound + 1;

    if(height == 1){
        height = 3;
        bottom_bound = max(0,bottom_bound-1);
    }
    if(width == 1){
        width = 3;
        left_bound = max(0,left_bound-1);
    }


}

void Shortest_Path::Construct_Edge(){
    
    // cout<<"AdjList size is "<<AdjList.size()<<endl;
    // cout<<"width is "<<width<<endl;
    // cout<<"height is "<<height<<endl;
    // cout<<"left bound "<<left_bound<<endl;
    // cout<<"bottom bound "<<bottom_bound<<endl;
    Construct_H_Edge();
    Construct_V_Edge();
}

void Shortest_Path::Construct_H_Edge(){         // Construct horizontal edge
    // cout<<"Construct_H_Edge"<<endl;
    int z = 0;
    for(size_t x = 0 ; x < width-1 ; x++){
        for(size_t y = 0 ; y < height ; y++){
            AdjList[x][y].push_back( pair<SP_direction,double>  (p_x , (*row_map)(x+left_bound, y+bottom_bound, z) ) );
            AdjList[x+1][y].push_back( pair<SP_direction,double> (n_x , (*row_map)(x+left_bound, y+bottom_bound, z) ) );
        }
        
    }
}

void Shortest_Path::Construct_V_Edge(){         // Construct horizontal edge
    // cout<<"Construct_V_Edge"<<endl;

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
    inside_min_heap.resize( width , vector<bool>  (height,false));
    // start point 
    int s_x = source_x - left_bound;
    int s_y = source_y - bottom_bound;
    grid_point_pointer[s_x][s_y] -> distance = 0 ;
    grid_point_pointer[s_x][s_y] -> H_bend  = true;
    grid_point_pointer[s_x][s_y] -> V_bend  = true;
    grid_point_pointer[s_x][s_y] -> bend = 0;

    visited[s_x][s_y] = inside_min_heap[s_x][s_y] = true;
    
    // end point
    int t_x = target_x - left_bound;
    int t_y = target_y - bottom_bound;
    // MinHeap<Coordinate> min_Heap( grid_point_Min_Heap.size() );
    
    MinHeap_pointer<Coordinate> min_Heap( 0 );

    for (list<pair<SP_direction,double>>::iterator iter = AdjList[s_x][s_y].begin();iter != AdjList[s_x][s_y].end(); iter++) {
        SP_direction dir = (*iter).first;
        int p2_x = s_x;
        int p2_y = s_y;
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
        // cout << "p2 x is "<<p2_x<<" p2 y is "<<p2_y<<endl; 
        inside_min_heap[p2_x][p2_y] = true;
        Relax( grid_point_pointer[s_x][s_y], grid_point_pointer[p2_x][p2_y], (*iter).second);
        min_Heap.insert( (grid_point_pointer[p2_x][p2_y]) );
    }
    // min_Heap.print_heap();
    
    // priority_queue <Coordinate,vector<Coordinate>,Coordinate_Compartor> min_Heap(grid_point.begin(),grid_point.end());
    
    // cout<<"min heap' size is "<<min_Heap.size()<<endl;
    // Coordinate point_min = min_Heap.top();
    // cout<<c_min.distance<<endl;
    while (!min_Heap.empty()) {
        // cout<<"before extractmin"<<endl;
        // min_Heap.print_heap();
        Coordinate* point_min = min_Heap.ExtractMin();
        // cout<<"after extractmin"<<endl;
        // min_Heap.print_heap();

        int p1_x = point_min->x - left_bound;
        int p1_y = point_min->y - bottom_bound;
        // cout << "new extract min "<<endl;
        // cout << "p1 x is "<<p1_x<<" p1 y is "<<p1_y<<" distance is :"<< grid_point_pointer[p1_x][p1_y]->distance << endl; 
        visited[p1_x][p1_y] = true;
        
        // find end point
        if(visited[t_x][t_y] == true) break;
        
        //update neighbor distance 
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
            // cout << "p2 x is "<<p2_x<<" p2 y is "<<p2_y<< " distance is :"<< grid_point_pointer[p2_x][p2_y]->distance << endl; 
            // cout << "visited: "<< visited[p2_x][p2_y] << " inside_min_heap : " <<inside_min_heap[p2_x][p2_y]<<endl;
            if(visited[p2_x][p2_y] == true) continue;

            bool D_bool = Relax( grid_point_pointer[p1_x][p1_y], grid_point_pointer[p2_x][p2_y], (*iter).second);
            if(inside_min_heap[p2_x][p2_y] == true ){
                if(D_bool){
                    // cout<<"decrease"<<endl;
                    // cout << "idx"<< grid_point_pointer[p2_x][p2_y]->index << endl; 
                    min_Heap.Decrease_key( (grid_point_pointer[p2_x][p2_y]));
                }
            }
            else{
                // cout<<"insert"<<endl;
                inside_min_heap[p2_x][p2_y] = true;
                min_Heap.insert( (grid_point_pointer[p2_x][p2_y]));
                // min_Heap.print_heap();
                
            }
        }
        // return;
    }
    // std::cout << "\nprint predecessor:\n";
    // PrintDataArray(predecessor);
    // std::cout << "\nprint distance:\n";
    // PrintDataArray(distance);
}

// void Shortest_Path::Dijkstra(){


//     visited.resize( width , vector<bool>  (height,false));  // initializa visited[] as {0,0,0,...,0}
//     inside_min_heap.resize( width , vector<bool>  (height,false));
//     int s_x = source_x - left_bound;
//     int s_y = source_y - bottom_bound;
//     // start point 
//     grid_point_pointer[s_x][s_y] -> distance = 0 ;
//     visited[s_x][s_y] = inside_min_heap[s_x][s_y] = true;
//     // MinHeap<Coordinate> min_Heap( grid_point_Min_Heap.size() );
    
//     MinHeap<Coordinate> min_Heap( 0 );

//     for (list<pair<SP_direction,double>>::iterator iter = AdjList[s_x][s_y].begin();iter != AdjList[s_x][s_y].end(); iter++) {
//         SP_direction dir = (*iter).first;
//         int p2_x = s_x;
//         int p2_y = s_y;
//         switch(dir){
//             case(p_x):{
//                 ++p2_x;
//                 break;
//             }
//             case(n_x):{
//                 --p2_x;
//                 break;
//             }
//             case(p_y):{
//                 ++p2_y;
//                 break;
//             }
//             case(n_y):{
//                 --p2_y;
//                 break;
//             }
//         }
//         cout << "p2 x is "<<p2_x<<" p2 y is "<<p2_y<<endl; 
//         inside_min_heap[p2_x][p2_y] = true;
//         Relax( grid_point_pointer[s_x][s_y], grid_point_pointer[p2_x][p2_y], (*iter).second);
//         min_Heap.insert( (*grid_point_pointer[p2_x][p2_y]));
//     }
//     min_Heap.print_heap();
    
//     // priority_queue <Coordinate,vector<Coordinate>,Coordinate_Compartor> min_Heap(grid_point.begin(),grid_point.end());
    
//     // cout<<"min heap' size is "<<min_Heap.size()<<endl;
//     // Coordinate point_min = min_Heap.top();
//     // cout<<c_min.distance<<endl;
//     while (!min_Heap.empty()) {
//         cout<<"before extractmin"<<endl;
//         min_Heap.print_heap();
//         Coordinate point_min = min_Heap.ExtractMin();
//         cout<<"after extractmin"<<endl;
//         min_Heap.print_heap();
//         // int p1_id = point_min.id(width,height);
//         // cout << p1_id<<endl;
//         // return;
//         int p1_x = point_min.x - left_bound;
//         int p1_y = point_min.y - bottom_bound;
//         cout << "new extract min "<<endl;
//         cout << "p1 x is "<<p1_x<<" p1 y is "<<p1_y<<" distance is :"<< grid_point_pointer[p1_x][p1_y]->distance << endl; 
//         visited[p1_x][p1_y] = true;
//         for (list<pair<SP_direction,double>>::iterator iter = AdjList[p1_x][p1_y].begin();
//             iter != AdjList[p1_x][p1_y].end(); iter++) {
//             SP_direction dir = (*iter).first;
//             int p2_x = p1_x;
//             int p2_y = p1_y;
//             switch(dir){
//                 case(p_x):{
//                     ++p2_x;
//                     break;
//                 }
//                 case(n_x):{
//                     --p2_x;
//                     break;
//                 }
//                 case(p_y):{
//                     ++p2_y;
//                     break;
//                 }
//                 case(n_y):{
//                     --p2_y;
//                     break;
//                 }
//             }
//             cout << "p2 x is "<<p2_x<<" p2 y is "<<p2_y<< " distance is :"<< grid_point_pointer[p2_x][p2_y]->distance << endl; 
//             // cout << "visited: "<< visited[p2_x][p2_y] << " inside_min_heap : " <<inside_min_heap[p2_x][p2_y]<<endl;
//             if(visited[p2_x][p2_y] == true) continue;

//             bool D_bool = Relax( grid_point_pointer[p1_x][p1_y], grid_point_pointer[p2_x][p2_y], (*iter).second);
//             if(inside_min_heap[p2_x][p2_y] == true ){
//                 if(D_bool){
//                     cout<<"decrease"<<endl;
//                     cout << "idx"<< grid_point_pointer[p2_x][p2_y]->index << endl; 
//                     min_Heap.Decrease_key( (*grid_point_pointer[p2_x][p2_y]));
//                 }
//             }
//             else{
//                 cout<<"insert"<<endl;
//                 inside_min_heap[p2_x][p2_y] = true;
//                 min_Heap.insert( (*grid_point_pointer[p2_x][p2_y]));
//                 min_Heap.print_heap();
                
//             }
//         }
//     }
// }

bool Shortest_Path::Relax( Coordinate* from, Coordinate* to, double weight ){
    
    if (to->distance > from->distance + weight) {
        to->distance = from->distance + weight;
        to->predecessor = from;
        int f_bend = from->bend;

        if(from->x == to->x){       //V_bend
            to->V_bend = true;
            to->V_predecessor = from;
            if(!from->V_bend){
                f_bend++;
            }
        }
        else{                       //H_bend
            to->H_bend = true;
            to->H_predecessor = from;
            if(!from->H_bend){
                f_bend++;
            }
        }
        to->bend = f_bend;
        return true;
    }

    if( to->distance == from->distance + weight){
        bool H_dir , V_dir;
        H_dir = V_dir = false;
        int f_bend = from->bend;
        if(from->x == to->x){       //V_bend
            V_dir = true;
            if(!from->V_bend){
                f_bend++;
            }
        }
        else{                       //H_bend
            H_dir = true;
            if(!from->H_bend){
                f_bend++;
            }
        }
        
        if(to->bend >= f_bend){
            to->bend = f_bend ;
            if(H_dir){
                to->H_bend = true;
                to->H_predecessor = from;
            }
            if(V_dir){
                to->V_bend = true;
                to->V_predecessor = from;
            }
        } 
        return false;
    }
    
}

Coordinate* Shortest_Path::End_point(){
    int t_x = target_x - left_bound;
    int t_y = target_y - bottom_bound;
    return grid_point_pointer[t_x][t_y];
}

void Shortest_Path::Build_the_path_old(){
    int temp_z = 0 ;
    target_bend = new Bend(target_x,target_y,temp_z);
    bool dir = true;            // the same direction of last coordinate
    bool H_dir ;                // the Horizontal direction
    bool last_H_dir;            // last path Horizontal direction
    int t_x = target_x - left_bound;
    int t_y = target_y - bottom_bound;

    int s_x = source_x - left_bound;
    int s_y = source_y - bottom_bound;
    
    Coordinate* temp = grid_point_pointer[t_x][t_y];
    Coordinate* n_temp;
    Bend* cur_bend = target_bend;
    Bend* n_bend;

    n_temp = temp->predecessor;
    if(n_temp->x == temp->x ){ last_H_dir = false; }
    
    else {last_H_dir = true;}

    while(temp != grid_point_pointer[s_x][s_y]){
        n_temp = temp->predecessor;
        // cout<<*temp<<endl;
        if(n_temp->x == temp->x ){
           H_dir = false; 
        }
        else {
            H_dir = true;
        }
        if( last_H_dir != H_dir ){
            last_H_dir = H_dir;
            n_bend = new Bend(temp->x, temp->y, temp->z ,cur_bend);
            cur_bend -> set_next( n_bend );
            cout<<"Bend"<<endl;
            cur_bend = n_bend;
            // cur_bend -> print();
        }
        temp = n_temp;
    }
    target_bend = target_bend -> get_next();
    target_bend -> set_prev(NULL);
    
    n_bend = new Bend(temp->x, temp->y, temp->z ,cur_bend);
    cur_bend -> set_next( n_bend );
    // n_bend -> print();

}

void Shortest_Path::Build_the_path(){
    int temp_z = 0 ;
    target_bend = new Bend(target_x,target_y,temp_z);
    bool dir = true;            // the same direction of last coordinate
    bool H_dir ;                // the Horizontal direction
    bool last_H_dir;            // last path Horizontal direction
    int t_x = target_x - left_bound;
    int t_y = target_y - bottom_bound;

    int s_x = source_x - left_bound;
    int s_y = source_y - bottom_bound;
    
    Coordinate* temp = grid_point_pointer[t_x][t_y];
    Coordinate* n_temp;
    if(temp->H_bend == true){
        n_temp = temp->H_predecessor;
        last_H_dir = true;
    }
    else{
        n_temp = temp->V_predecessor;
        last_H_dir = false;
    }
    Bend* cur_bend = target_bend;
    Bend* n_bend;

    while(temp != grid_point_pointer[s_x][s_y]){
        if(temp->H_bend == true){
            n_temp = temp->H_predecessor;
            H_dir = true;
        }
        else{
            n_temp = temp->V_predecessor;
            H_dir = false;
        }
        // cout<<*temp<<endl;
        // if(n_temp->x == temp->x ){
        //    H_dir = false; 
        // }
        // else {
        //     H_dir = true;
        // }
        if( last_H_dir != H_dir ){
            last_H_dir = H_dir;
            n_bend = new Bend(temp->x, temp->y, temp->z , NULL, cur_bend);
            assert(n_bend != NULL);
            cur_bend -> set_prev( n_bend );
            // cout<<"Bend"<<endl;
            cur_bend = n_bend;
            // cur_bend -> print();
        }
        temp = n_temp;
    }
    // target_bend->print();
    // target_bend = target_bend -> get_next();
    if( target_bend == NULL){
        cerr << "QAQ"<<endl;
        // exit(-1);
    }
    // target_bend -> set_prev(NULL);
    // return;
    
    n_bend = new Bend(temp->x, temp->y, temp->z, NULL, cur_bend);
    cur_bend -> set_prev( n_bend );
    // n_bend -> print();
    source_bend = n_bend;
    for( int i = 0 ; i < grid_point_pointer.size(); i++){
        for( int j = 0; j < grid_point_pointer[i].size(); j++){
            delete grid_point_pointer[i][j];
        }
    }
}

void Shortest_Path::reverse_path(){
//     Bend* first,second;
//     first = target_bend;
//     second = first->get_next();
//     while(second != NULL){
        
//     }
}