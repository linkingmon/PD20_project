#include "maze_router.h"
#include <assert.h>

using namespace std;

// void Shortest_Path:

void Shortest_Path::InitializeSingleSource(){
  
    // Set_boundary();
    grid_point_pointer.resize( width , vector<Coordinate*>(height));
    AdjList.resize( width , vector<list<pair<SP_direction,double> > > (height));
    visited.resize( width , vector<bool> (height,false));
    int z = 0;
    for(int x = 0 ; x < width ; x++){
        for(int y = 0 ; y < height ; y++){
            double distance = 1000;
            Coordinate* temp = new Coordinate(x+left_bound, y+bottom_bound, z, distance);
            temp->estimate_distance = Estimate_distance_to_target(x+left_bound ,y+bottom_bound);
            grid_point_Min_Heap.push_back( (*temp) );
            grid_point_pointer[x][y] = ( (temp ) );
        }
    }

    Construct_Edge();
}

void Shortest_Path::InitializeMultiSource(){
  
    // Set_boundary();
    grid_point_pointer.resize( width , vector<Coordinate*>(height));
    AdjList.resize( width , vector<list<pair<SP_direction,double> > > (height));
    visited.resize( width , vector<bool> (height,false));
    set_of_start_point.resize( width , vector<bool> (height,false));
    set_of_end_point.resize( width , vector<bool> (height,false));

    int z = 0;
    for(int x = 0 ; x < width ; x++){
        for(int y = 0 ; y < height ; y++){
            double distance = 1000;
            Coordinate* temp = new Coordinate(x+left_bound, y+bottom_bound, z, distance);
            temp->estimate_distance = Estimate_distance_to_target(x+left_bound ,y+bottom_bound);
            grid_point_Min_Heap.push_back( (*temp) );
            grid_point_pointer[x][y] = ( (temp ) );
        }
    }

    Construct_Edge();
    cout<<"done"<<endl;
}

void Shortest_Path::Setup_multi_source(vector<pair<int,int>> a, MinHeap_pointer<Coordinate>& min_Heap){
    // cout<<a.size()<<endl;s
    for(size_t i = 0; i < a.size(); i++){
        int x = a[i].first - left_bound;
        int y = a[i].second - bottom_bound;
        // cout<<x<<" "<<y<<endl;
        set_of_start_point[x][y] = true;
    
        grid_point_pointer[x][y] -> distance = 0 ;
        grid_point_pointer[x][y] -> H_bend  = true;
        grid_point_pointer[x][y] -> V_bend  = true;
        grid_point_pointer[x][y] -> bend = 0;

        visited[x][y] = inside_min_heap[x][y] = true;
        min_Heap.insert( (grid_point_pointer[x][y]) );
    }
}

void Shortest_Path::Setup_multi_end(vector<pair<int,int>> a){
    for(size_t i = 0; i < a.size(); i++){
        int x = a[i].first - left_bound;
        int y = a[i].second - bottom_bound;
        set_of_end_point[x][y] = true;
    }
}
int Shortest_Path::Estimate_distance_to_target(int x, int y){
    // x and y is original coordinate
    return abs(target_x - x) + abs(target_y - y);
}

void Shortest_Path::Set_boundary(){
    cout<<"Set Boundary"<<endl;
    height = abs(target_x-source_x)* expand_factor;
    width =  abs(target_y-source_y)* expand_factor;
    int Total_height = col_map->get_row();
    int Total_width = row_map->get_col();

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
        bottom_bound = min(Total_height - height +1, bottom_bound);
    }
    if(width == 1){
        width = 3;
        left_bound = min(Total_width - width +1, left_bound);
    }

    // cout<<"width: "<<width<<" "<<Total_width<<endl;
    // cout<<"Height: "<<height<<" "<<Total_height<<endl;
    // cout<<"left"<<left_bound<<endl;
    // cout<<"bottom"<<bottom_bound<<endl;
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
    cout<<"Construct_H_Edge"<<endl;
    int z = 0;
    // cout<<AdjList.size()<<endl;
    // cout<<AdjList[0].size()<<endl;
    for(size_t x = 0 ; x < width-1 ; x++){
        for(size_t y = 0 ; y < height ; y++){
            AdjList[x][y].push_back( pair<SP_direction,double>  (p_x , (*row_map)(x+left_bound, y+bottom_bound, z) ) );
            AdjList[x+1][y].push_back( pair<SP_direction,double> (n_x , (*row_map)(x+left_bound, y+bottom_bound, z) ) );
        }
    }
}

void Shortest_Path::Construct_V_Edge(){         // Construct horizontal edge
    cout<<"Construct_V_Edge"<<endl;
    // col_map -> print_congestion();
    int z = 0;
    for(size_t x = 0 ; x < width ; x++){
        for(size_t y = 0 ; y < height-1 ; y++){
            AdjList[x][y].push_back( pair<SP_direction,double> (p_y , (*col_map)(x+left_bound, y+bottom_bound, z) ) );
            AdjList[x][y+1].push_back( pair<SP_direction,double> (n_y , (*col_map)(x+left_bound, y+bottom_bound, z) ) );
        } 
    }
}

void Shortest_Path::Dijkstra_multi(){
    visited.resize( width , vector<bool>  (height,false));  // initializa visited[] as {0,0,0,...,0}
    inside_min_heap.resize( width , vector<bool>  (height,false));
    // start point 
    int s_x = source_x - left_bound;
    int s_y = source_y - bottom_bound;
    cout<<s_x<<" "<<s_y<<endl;
    grid_point_pointer[s_x][s_y] -> distance = 0 ;
    grid_point_pointer[s_x][s_y] -> H_bend  = true;
    grid_point_pointer[s_x][s_y] -> V_bend  = true;
    grid_point_pointer[s_x][s_y] -> bend = 0;
    set_of_start_point[s_x][s_y] = true;
    visited[s_x][s_y] = inside_min_heap[s_x][s_y] = true;
    
    // end point
    int t_x = target_x - left_bound;
    int t_y = target_y - bottom_bound;
    // MinHeap<Coordinate> min_Heap( grid_point_Min_Heap.size() );
    MinHeap_pointer<Coordinate> min_Heap( 0 );
    Setup_multi_source( source_vector, min_Heap);
    Setup_multi_end( end_vector );
    // min_Heap.insert( (grid_point_pointer[s_x][s_y]) );
    // for (list<pair<SP_direction,double>>::iterator iter = AdjList[s_x][s_y].begin();iter != AdjList[s_x][s_y].end(); iter++) {
    //     SP_direction dir = (*iter).first;
    //     int p2_x = s_x;
    //     int p2_y = s_y;
    //     switch(dir){
    //         case(p_x):{
    //             ++p2_x;
    //             break;
    //         }
    //         case(n_x):{
    //             --p2_x;
    //             break;
    //         }
    //         case(p_y):{
    //             ++p2_y;
    //             break;
    //         }
    //         case(n_y):{
    //             --p2_y;
    //             break;
    //         }
    //     }
    //     // cout << "p2 x is "<<p2_x<<" p2 y is "<<p2_y<<endl; 
    //     inside_min_heap[p2_x][p2_y] = true;
    //     Relax( grid_point_pointer[s_x][s_y], grid_point_pointer[p2_x][p2_y], (*iter).second);
    //     min_Heap.insert( (grid_point_pointer[p2_x][p2_y]) );
    // }

    while (!min_Heap.empty()) {
        // cout<<"before extractmin"<<endl;
        min_Heap.print_heap();
        Coordinate* point_min = min_Heap.ExtractMin();
        // cout<<"after extractmin"<<endl;
        // min_Heap.print_heap();

        int p1_x = point_min->x - left_bound;
        int p1_y = point_min->y - bottom_bound;
        // cout << endl;
        cout << "new extract min "<<endl;
        cout << "p1 x is "<<p1_x + left_bound <<" p1 y is "<<p1_y + bottom_bound<<" distance is :"<< grid_point_pointer[p1_x][p1_y]->distance << endl; 
        visited[p1_x][p1_y] = true;
        
        // find end point
        if(set_of_end_point[p1_x][p1_y] == true) {
            final_tx = p1_x + left_bound;
            final_ty = p1_y + bottom_bound;
            cout<<"end point"<<endl;
            break;
        }
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
            if(visited[p2_x][p2_y] == true) continue;
            // cout<<" update distance "<< (*iter).second <<endl;
            bool D_bool = Relax( grid_point_pointer[p1_x][p1_y], grid_point_pointer[p2_x][p2_y], (*iter).second);
            // cout << endl;
            // cout << "update neighbor"<<endl;
            // cout << "p2 x is "<<p2_x + left_bound<<" p2 y is "<<p2_y + bottom_bound<< " distance is :"<< grid_point_pointer[p2_x][p2_y]->distance << endl; 
            // cout << "visited: "<< visited[p2_x][p2_y] << " inside_min_heap : " <<inside_min_heap[p2_x][p2_y]<<endl;
            // cout << endl;

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
    }
}

void Shortest_Path::Dijkstra(){

    
    visited.resize( width , vector<bool>  (height,false));  // initializa visited[] as {0,0,0,...,0}
    inside_min_heap.resize( width , vector<bool>  (height,false));
    // start point 
    int s_x = source_x - left_bound;
    int s_y = source_y - bottom_bound;
    cout<<s_x<<" "<<s_y<<endl;
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
        // cout << endl;
        // cout << "new extract min "<<endl;
        // cout << "p1 x is "<<p1_x + left_bound <<" p1 y is "<<p1_y + bottom_bound<<" distance is :"<< grid_point_pointer[p1_x][p1_y]->distance << endl; 
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
            if(visited[p2_x][p2_y] == true) continue;
            // cout<<" update distance "<< (*iter).second <<endl;
            bool D_bool = Relax( grid_point_pointer[p1_x][p1_y], grid_point_pointer[p2_x][p2_y], (*iter).second);
            // cout << endl;
            // cout << "update neighbor"<<endl;
            // cout << "p2 x is "<<p2_x + left_bound<<" p2 y is "<<p2_y + bottom_bound<< " distance is :"<< grid_point_pointer[p2_x][p2_y]->distance << endl; 
            // cout << "visited: "<< visited[p2_x][p2_y] << " inside_min_heap : " <<inside_min_heap[p2_x][p2_y]<<endl;
            // cout << endl;

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
    
    // assert(visited[t_x][t_y] == true) ;
    // std::cout << "\nprint predecessor:\n";
    // PrintDataArray(predecessor);
    // std::cout << "\nprint distance:\n";
    // PrintDataArray(distance);
}

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
            assert( from->y == to->y );
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
            assert( from->y == to->y );
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

    // cout<< "from ( " <<source_x <<" , "<<source_y<<" )"<<endl;
    // cout<< "to   ( " <<target_x <<" , "<<target_y<<" )"<<endl;

    // check the first path is H/V
    // cout<<"build path"<<endl;
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
    
    // cout<<*temp<<endl;
    // cout<<*n_temp<<endl;
    assert(n_temp != NULL);
    // cout<<"while loop"<<endl;
    while(temp != grid_point_pointer[s_x][s_y]){
        if(temp->H_bend == true){
            n_temp = temp->H_predecessor;
            H_dir = true;
            cout<<"H Bend"<<endl;
            cout<<*temp<<endl;
            cout<<*n_temp<<endl;
            assert( n_temp -> y == temp -> y);
        }
        else{
            n_temp = temp->V_predecessor;
           
            cout<<"V Bend"<<endl;
            cout<<*temp<<endl;
            cout<<*n_temp<<endl;
            assert( n_temp -> x == temp -> x);
            H_dir = false;
        }
        // cout<<*temp<<endl;
        // if(n_temp->x == temp->x ){
        //    H_dir = false; 
        // }
        // else {
        //     H_dir = true;
        // }
        assert(n_temp != NULL);

        if( last_H_dir != H_dir ){
            last_H_dir = H_dir;
            n_bend = new Bend(temp->x, temp->y, temp->z , NULL, cur_bend);
            assert(cur_bend != NULL);            
            assert(n_bend != NULL);
            cur_bend -> set_prev( n_bend );
            // cout<<"Bend"<<endl;
            cur_bend = n_bend;
            // cur_bend -> print();
        }
        temp = n_temp;

    }
    // cout<<*temp<<endl;
    // target_bend->print();
    // target_bend = target_bend -> get_next();
    if( target_bend == NULL){
        cerr << "QAQ"<<endl;
        // exit(-1);
    }
    // target_bend -> set_prev(NULL);
    // return;
    // cout<<"last bend"<<endl;
    n_bend = new Bend(temp->x, temp->y, temp->z, NULL, cur_bend);
    cur_bend -> set_prev( n_bend );
    // n_bend -> print();
    source_bend = n_bend;
    for( int i = 0 ; i < grid_point_pointer.size(); i++){
        for( int j = 0; j < grid_point_pointer[i].size(); j++){
            delete grid_point_pointer[i][j];
        }
    }
    assert(visited[t_x][t_y] == true) ;
}

void Shortest_Path::Build_the_path_multi(){
    int temp_z = 0 ;
    target_bend = new Bend(target_x,target_y,temp_z);
    bool dir = true;            // the same direction of last coordinate
    bool H_dir ;                // the Horizontal direction
    bool last_H_dir;            // last path Horizontal direction
    int t_x = final_tx - left_bound;
    int t_y = final_ty - bottom_bound;

    int s_x = source_x - left_bound;
    int s_y = source_y - bottom_bound;
    
    Coordinate* temp = grid_point_pointer[t_x][t_y];
    Coordinate* n_temp;
    
    // cout<<"good"<<endl;
    // cout<<*temp<<endl;
    // cout<<"bad"<<endl;
    // cout<< "from ( " <<source_x <<" , "<<source_y<<" )"<<endl;
    // cout<< "to   ( " <<target_x <<" , "<<target_y<<" )"<<endl;

    // cout<< "from ( " <<s_x <<" , "<<s_y<<" )"<<endl;
    // cout<< "to   ( " <<t_x <<" , "<<t_y<<" )"<<endl;

    // check the first path is H/V
    // cout<<"build path"<<endl;
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
    
    // cout<<*temp<<endl;
    // cout<<*n_temp<<endl;
    assert(n_temp != NULL);
    // cout<<"while loop"<<endl;
    while( !is_start_point(temp) ){
        if(temp->H_bend == true){
            n_temp = temp->H_predecessor;
            H_dir = true;
            cout<<"H Bend"<<endl;
            cout<<*temp<<endl;
            cout<<*n_temp<<endl;
            assert( n_temp -> y == temp -> y);
        }
        else{
            n_temp = temp->V_predecessor;
            cout<<"V Bend"<<endl;
            cout<<*temp<<endl;
            cout<<*n_temp<<endl;
            assert( n_temp -> x == temp -> x);
            H_dir = false;
        }
        // cout<<*temp<<endl;
        // if(n_temp->x == temp->x ){
        //    H_dir = false; 
        // }
        // else {
        //     H_dir = true;
        // }
        assert(n_temp != NULL);

        if( last_H_dir != H_dir ){
            last_H_dir = H_dir;
            n_bend = new Bend(temp->x, temp->y, temp->z , NULL, cur_bend);
            assert(cur_bend != NULL);            
            assert(n_bend != NULL);
            cur_bend -> set_prev( n_bend );
            // cout<<"Bend"<<endl;
            cur_bend = n_bend;
            // cur_bend -> print();
        }
        temp = n_temp;
    }
    // cout<<*temp<<endl;
    // target_bend->print();
    // target_bend = target_bend -> get_next();
    if( target_bend == NULL){
        cerr << "QAQ"<<endl;
        // exit(-1);
    }
    // target_bend -> set_prev(NULL);
    // return;
    // cout<<"last bend"<<endl;
    n_bend = new Bend(temp->x, temp->y, temp->z, NULL, cur_bend);
    cur_bend -> set_prev( n_bend );
    // n_bend -> print();
    source_bend = n_bend;
    final_sx = temp->x;
    final_sy = temp->y;

    for( int i = 0 ; i < grid_point_pointer.size(); i++){
        for( int j = 0; j < grid_point_pointer[i].size(); j++){
            delete grid_point_pointer[i][j];
        }
    }
    assert(visited[t_x][t_y] == true) ;
}

bool Shortest_Path::is_start_point(Coordinate* s){
    int x = s -> x - left_bound;
    int y = s -> y - bottom_bound;
    return set_of_start_point[x][y];
}

void Shortest_Path::reverse_path(){
//     Bend* first,second;
//     first = target_bend;
//     second = first->get_next();
//     while(second != NULL){
        
//     }
}

void Maze_router::SP_setup(){
    SP.set_start( start_point );
    SP.set_end( end_point );
}

void Maze_router::Initialize(){
    two_pin_net_id.resize(width, vector<int>(height,-1));
    branch_map.resize(width, vector<branch*>(height,NULL));
    Initialize_start();
    Initialize_end();
    SP_setup();
}

void Maze_router::Initialize_start(){
    two_pin_net_id[start_x - left_bound][start_y - bottom_bound] = 0;
    start_point.push_back( make_pair(start_x, start_y));
    cout<<"start tree"<<endl;
    if(start_tree == NULL) return;
    vector<two_pin_net*> netlist = start_tree -> netlist;
    for(size_t i = 0; i < netlist.size(); i++){
        Bend* source = netlist[i] -> get_source();
        Bend* n_bend = source -> get_next();
        branch* s1 = netlist[i] -> b_source;
        branch* s2 = netlist[i] -> b_target;
        int s1_x = s1->_x;
        int s1_y = s1->_y;
        int s2_x = s2->_x;
        int s2_y = s2->_y;
        branch_map[s1_x][s1_y] = s1;
        branch_map[s2_x][s2_y] = s2;
        while(n_bend != NULL){
            int x1 = source -> _x - left_bound;
            int y1 = source -> _y - bottom_bound;
            int x2 = n_bend -> _x - left_bound;
            int y2 = n_bend -> _y - bottom_bound;
            source = n_bend;
            n_bend = source -> get_next();
            if(x1 > x2) swap(x1,x2);
            if(y1 > y2) swap(y1,y2);
            if(x1 == x2){
                if( x1 < left_bound || x1 > (width - 1))
                    continue;
                if(y1 < bottom_bound) y1 = bottom_bound;
                if(y2 > height - 1) y2 = height - 1 ;
            }
            if(y1 == y2){
                if( y1 < bottom_bound || y2 > (height - 1))
                    continue;
                if(x1 < left_bound) x1 = left_bound;
                if(x2 > width) x2 = width - 1;
            }
            for(int x = x1; x <= x2; x++){
                for(int y = y1; y <= y2; y++){
                    two_pin_net_id[x][y] = i;
                    start_point.push_back( make_pair(x + left_bound, y + bottom_bound));
                }
            }
        }
    }
    
}

void Maze_router::Initialize_end(){
    two_pin_net_id[end_x - left_bound][end_y - bottom_bound] = 0;
    end_point.push_back( make_pair(end_x, end_y));
    if(end_tree == NULL) return;
    vector<two_pin_net*> netlist = end_tree -> netlist;
    for(size_t i = 0; i < netlist.size(); i++){
        Bend* source = netlist[i] -> get_source();
        Bend* n_bend = source -> get_next();
        while(n_bend != NULL){
            int x1 = source -> _x - left_bound;
            int y1 = source -> _y - bottom_bound;
            int x2 = n_bend -> _x - left_bound;
            int y2 = n_bend -> _y - bottom_bound;
            source = n_bend;
            n_bend = source -> get_next();
            if(x1 > x2) swap(x1,x2);
            if(y1 > y2) swap(y1,y2);
            if(x1 == x2){
                if( x1 < left_bound || x1 > (width - 1))
                    continue;
                if(y1 < bottom_bound) y1 = bottom_bound;
                if(y2 > height - 1) y2 = height - 1 ;
            }
            if(y1 == y2){
                if( y1 < bottom_bound || y2 > (height - 1))
                    continue;
                if(x1 < left_bound) x1 = left_bound;
                if(x2 > width) x2 = width - 1;
            }
            for(int x = x1; x <= x2; x++){
                for(int y = y1; y <= y2; y++){
                    two_pin_net_id[x][y] = i;
                    end_point.push_back( make_pair(x + left_bound, y + bottom_bound));
                }
            }
        }
    }
}

void Maze_router::Connect_two_tree(){
    int final_sx = SP.final_sx;
    int final_sy = SP.final_sy;
    int final_tx = SP.final_tx;
    int final_ty = SP.final_ty;
    // b_start = new branch(final_sx,final_sy,0, INT_MAX);
    // b_end = new branch(final_tx,final_ty,0, INT_MAX);
    
    if(branch_map[final_sx][final_sy] == NULL){
        b_start = new branch(final_sx,final_sy,0, INT_MAX);
        adjust_tree(start_tree,final_sx,final_sy,b_start);
    }
    else {
        b_start = branch_map[final_sx][final_sy];
    }
    if(branch_map[final_tx][final_ty] == NULL){
        b_end = new branch(final_tx,final_ty,0, INT_MAX);
        adjust_tree(end_tree,final_tx,final_ty,b_end);       
    }
    else{
        b_end = branch_map[final_tx][final_ty];
    }
    
    new_net = new two_pin_net(b_start,b_end);
    new_net -> set_source(SP.source());

    final_tree = Connect_two_tree(start_tree, end_tree);
    final_tree -> netlist.push_back(new_net);
}

SubTree* Maze_router::Connect_two_tree(SubTree* a, SubTree* b){
    SubTree* Main_tree = a;
    SubTree* little_tree = b;
    if(a->netlist.size() < b->netlist.size() ){
        swap(Main_tree,little_tree);
    }
    for(size_t i = 0; i < little_tree->netlist.size(); i++){
        two_pin_net* temp_net = little_tree->netlist[i];
        temp_net -> b_source -> tree = Main_tree;
        temp_net -> b_target -> tree = Main_tree;
    }
    Main_tree -> netlist.insert( Main_tree -> netlist.end(), little_tree->netlist.begin(), little_tree->netlist.end() );
    delete little_tree;
    return Main_tree;
}

void Maze_router::adjust_tree(SubTree* T, int x, int y, branch* new_branch){
    int id = two_pin_net_id[x - left_bound][y - bottom_bound];
    assert(id >= 0);
    branch* a = T -> netlist[id] -> b_source;
    branch* b = T -> netlist[id] -> b_target;
    // new_branch = new branch(x,y,0);
    Bend* origin_s = T -> netlist[id] -> get_source();
    Bend* n_bend = origin_s -> get_next();
    while( n_bend -> _x != x && n_bend -> _y != y){
        origin_s = n_bend;
        n_bend = origin_s -> get_next();
    }
    // if( n_bend -> _x == x){
    // }
    // if( n_bend -> _y == y)
    
    Bend* tapping_point_1 = new Bend(x,y,0,n_bend);
    Bend* tapping_point_2 = new Bend(x,y,0);
    n_bend -> set_next(tapping_point_1);

    n_bend = n_bend -> get_next();
    n_bend -> set_prev(tapping_point_2);
    tapping_point_2 -> set_next(n_bend);
    
    two_pin_net* n1 = new two_pin_net(a,new_branch);
    n1 -> set_source( tapping_point_1 );
    two_pin_net* n2 = new two_pin_net(b,new_branch);
    n2 -> set_source( tapping_point_2 );
    T -> netlist.push_back(n1);
    T -> netlist.push_back(n2);
    T -> netlist.erase( T -> netlist.begin() + id );
}