#include <stdio.h>
#include <map>
#include <algorithm>
#include <queue>
#include "router.h"
#include "net.h"
#include "struc.h"
#include "congestion.h"
#include <assert.h>
#include <math.h>
#include <cmath>
#include <limits.h>
using namespace std;

// to do list
// 2D projection check
// expansion check
// stenier structrue check 
// congestion map update when path is determined (add/exclude) check
// layer assignment

ostream &operator<<(ostream &s, pair<int,int> p ) { 
    s << "("<< p.first << ", " << p.second << ")"; 
    return s; 
} 

void Router::route(){
    
    cout << "Routing ..." << endl;
    // cout<<(*row_map)(2,3,1)<<endl;

    // cost_row_map->print_congestion();
    cost_col_map->print_congestion();
    
    construct_grid_map();
    construct_supply_demand_map();
    // supply_grid_map->print_congestion();
    // demand_grid_map->print_congestion();

    projection_to_2D();
    supply_from_grid_to_edge();
    x_expand_factor.resize(_placement->_boundary_width-1,1.5);
    y_expand_factor.resize(_placement->_boundary_height-1,1.5);
    x_expand_result.resize(_placement->_boundary_width);
    y_expand_result.resize(_placement->_boundary_height);
    
    // for(int i = 0 ; i< 10;i++){
        
    //     Shortest_Path test( 1, 1, 0, 3, 3, 0, 1, cost_row_map, cost_col_map, demand_grid_map );
    //     test.Dijkstra();
    //     test.Build_the_path();
    // }
    cout<<" contruct congestion map"<<endl;
    cout<<endl;
    construct_congestion_map();
    cout<<"A star serach routing"<<endl;
    cout<<endl;
    for(int i = 0; i < 5 ; i++){
        cout<<"the "<<i<<" times A star"<<endl;
        A_star_search_routing();
    }
    cout<<"layer assignment"<<endl;
    cout<<endl; 
    layer_assignment(); 
    check_demand();
    // layer_range.print();    

    // supply_grid_map->print_congestion();
    // demand_grid_map->print_congestion();
    // write_result_to_cmd();
    
}

void Router::construct_grid_map(){
    int width =  _placement->_boundary_width;
    int height = _placement->_boundary_height;
    int MCell_num = _placement->_numMasterCell;
    int cell_num = _placement->_numCells;
    int norm_x_factor = _placement->_leftBoundary;
    int norm_y_factor = _placement->_bottomBoundary;

    // initial Grid
    for(size_t x = 0 ; x < width ; x++){
        for (size_t y = 0 ; y < height; y++){
            grid_map[x][y] = new Grid(x,y,MCell_num);
        }
    }

    // put Cell in grid
    for(size_t i = 0 ; i < cell_num ; i++){
        Master* m_type = _placement->getCell(i)->get_master();
        int x = _placement->getCell(i)->getx() - norm_x_factor;
        int y = _placement->getCell(i)->gety() - norm_y_factor;
        int Master_id = m_type->getId();
        grid_map[x][y]->add_MCell( Master_id );
        // ++(grid_map[x][y]->getMCell(Master_id));
        grid_map[x][y]->add_Cell(i);
    }

}

void Router::construct_supply_demand_map(){
    
    // supply map construct
    // supply_grid_map->cong_map_clear();
    int layer_num = _placement->_numLayers;
    int height = _placement->_boundary_height;
    int norm_y_factor = _placement->_leftBoundary;
    int width = _placement->_boundary_width;
    int norm_x_factor = _placement->_bottomBoundary;
    int norm_z_factor = 1 ;

    // defult supply
    // cout<<"default supply"<<endl;
    for(size_t layer = 0 ; layer < layer_num ; layer++){
        int default_supply = _placement->layers[layer]->get_supply();
        vector<vector<double>> one_layer_cong = (vector<vector<double>>(width , vector<double>(height,default_supply)) );
        supply_grid_map->set_congestion_map(layer,one_layer_cong);
    }

    // non_defult supply 
    // cout<<"non default F"<<endl;
    size_t non_default_num = _placement->_numNonDefault;
    for(size_t i = 0 ; i < non_default_num ; i++){
        int x = _placement->nondefault[i]->_x - norm_x_factor;
        int y = _placement->nondefault[i]->_y - norm_y_factor;
        int z = _placement->nondefault[i]->_z - norm_z_factor;
        int offset = _placement->nondefault[i]->_offset;
        // cout<<"x is "<<x<<" y is "<<y<<" z is "<<z<<" offset is " <<offset<<endl;
        (*supply_grid_map)(x,y,z) += offset;
    }
    
    // add block demand
    size_t cell_num = _placement->_numCells;
    for(size_t i = 0 ; i < cell_num ; i++){
        Master* m_type = _placement->getCell(i)->get_master();
        vector<int> block_layer_list = m_type->get_block_layer();
        vector<int> block_demand_list = m_type->get_block_demand();
        size_t block_num = block_layer_list.size();
        
        int x = _placement->getCell(i)->getx() - norm_x_factor;
        int y = _placement->getCell(i)->gety() - norm_y_factor;
        // cout<<"x is "<<x<<" y is "<<y<<endl;

        for (size_t j = 0 ; j < block_num ; j++){
            size_t block_layer = block_layer_list[j];
            // cout<<block_layer<<endl;
            // cout<<block_demand_list[j]<<endl;
            (*demand_grid_map)(x,y,block_layer) += block_demand_list[j];
        }
    }

    // add adjacent (Horizontal) and same Grid demand
    for(size_t x = 0 ; x < width-1 ; x++){        // Column index
        for(size_t y = 0 ; y < height ; y++){   // Row index
            map<int,int> temp_MClist = grid_map[x][y] -> get_MCell_list();
            map<int,int>::iterator temp_iter , same_iter , adj_iter ;
            
            map<int,int> adj_MClist = grid_map[x+1][y] -> get_MCell_list();
            for( temp_iter = temp_MClist.begin(); temp_iter != temp_MClist.end() ; temp_iter++ ){
                int MC1_id = temp_iter->first;
                int MC1_num = temp_iter->second;
                Master* m1 = _placement->getMCell(MC1_id);
                vector<ExtraDemand> sameDemand = m1->get_sameDemand();
                vector<ExtraDemand> adjHDemand = m1->get_adjHDemand();
                size_t s_num = sameDemand.size();
                size_t aH_num = adjHDemand.size();
                for(size_t i = 0 ; i < s_num ; i++){            //same grid demand
                    int MC2_id = sameDemand[i]._extraId;
                    if ( MC2_id < MC1_id)   continue;
                    same_iter = temp_MClist.find(MC2_id);
                    if(same_iter == temp_MClist.end()) continue;
                    int MC2_num = same_iter->second;
                    int layer = sameDemand[i]._layer;
                    int demand = sameDemand[i]._demand;
                    int MC_num = min(MC1_num,MC2_num);
                    (*demand_grid_map)(x,y,layer) += MC_num * demand;
                }

                
                for(size_t j = 0 ; j < aH_num ;j++){      // adj Horizontal grid demand
                    int MC2_id = adjHDemand[j]._extraId;
                    adj_iter = adj_MClist.find(MC2_id);
                    if(adj_iter == adj_MClist.end()) continue;
                    int MC2_num = adj_iter->second;
                    int layer = adjHDemand[j]._layer;
                    int demand = adjHDemand[j]._demand;
                    int MC_num = min(MC1_num,MC2_num);
                    (*demand_grid_map)(x,y,layer) += MC_num * demand;
                    (*demand_grid_map)(x+1,y,layer) += MC_num * demand;
                }
            }
        }
    }       
}

void Router::two_pin_net_L_routing(Pin* a  , Pin* b){
    
    Cell* c_a = _placement->getCell(a->getcellId());
    Cell* c_b = _placement->getCell(b->getcellId());
    int a_x = c_a->getx();
    int a_y = c_a->gety();
    int a_z = a->get_layer();
    int b_x = c_b->getx();
    int b_y = c_b->gety();
    int b_z = b->get_layer();

    branch* branch_a = new branch(a_x,a_y,a_z);
    branch* branch_b = new branch(b_x,b_y,b_z);
    two_pin_net* L_net = new two_pin_net(branch_a,branch_b);
    
    int z = 0;
    Bend* sink = new Bend(a_x,a_y,z); 
    L_net->set_source ( new Bend(a_x,a_y,z) );
    Bend* L_bend = L_route_2D(a_x,b_x,a_y,b_y,z);
    sink->set_next(L_bend);
    L_bend->set_prev(sink);

    twopin_netlist_L.push_back(L_net);
    
}

void Router::two_pin_net_Both_L_routing( two_pin_net* a , bool first_round){

    branch* branch_a = a->b_source;
    branch* branch_b = a->b_target;
    // two_pin_net<int>* L_net = new two_pin_net<int>(branch_a,branch_b);
    int a_x = branch_a->_x;
    int a_y = branch_a->_y;
    int a_z = branch_a->_z;
    int b_x = branch_b->_x;
    int b_y = branch_b->_y;
    int b_z = branch_b->_z;
    int z = 0;
    
    // cout<<"a: "<<a_x<<" "<<a_y<<" "<<endl;
    // cout<<"b: "<<b_x<<" "<<b_y<<" "<<endl;
    Bend* sink = new Bend(a_x,a_y,z); 
    Bend* target = new Bend(b_x,b_y,z);
    if(a_x == b_x){
        sink->set_next(target);
        if(first_round){
            Add_demand(sink,1);
            delete sink;
            delete target;
            return;
        }
        else{
            Exclude_demand(sink,1);
            return;
        }
    }
    if(a_y == b_y){
        sink->set_next(target);
        if(first_round){
            Add_demand(sink,1);
            delete sink;
            delete target;
            return;
        }
        else{
            Exclude_demand(sink,1);
            return;
        }
    }
    if(first_round){
        // H -> V
        Bend* H_bend = new Bend(b_x,a_y,z);
        sink->set_next(H_bend);
        H_bend->set_next(target);
        Add_demand(sink,0.5);
        
        // V -> H
        Bend* V_bend = new Bend(a_x,b_y,z);
        sink->set_next(V_bend);
        V_bend->set_next(target);
        Add_demand(sink,0.5);
        delete sink;
        delete H_bend;
        delete V_bend;
        delete target;
    }
    else{
        // H -> V
        Bend* H_bend = new Bend(b_x,a_y,z);
        sink->set_next(H_bend);
        H_bend->set_next(target);
        Exclude_demand(sink,0.5);
        
        // V -> H        
        sink = new Bend(a_x,a_y,z); 
        target = new Bend(b_x,b_y,z);
        Bend* V_bend = new Bend(a_x,b_y,z);
        sink->set_next(V_bend);
        V_bend->set_next(target);
        Exclude_demand(sink,0.5);
    }
}

void Router::two_pin_net_L_routing( two_pin_net* a ){

    branch* branch_a = a->b_source;
    branch* branch_b = a->b_target;
    // two_pin_net<int>* L_net = new two_pin_net<int>(branch_a,branch_b);
    int a_x = branch_a->_x;
    int a_y = branch_a->_y;
    int a_z = branch_a->_z;
    int b_x = branch_b->_x;
    int b_y = branch_b->_y;
    int b_z = branch_b->_z;
    int z = 0;


    Bend* sink = new Bend(a_x,a_y,z); 
    
    Bend* L_bend = L_route_2D(a_x,b_x,a_y,b_y,z);
    if( L_bend->_x == a_x && L_bend->_y == a_y){
        sink = L_bend;    
    }
    else{
        sink->set_next(L_bend);
        L_bend->set_prev(sink);
    }
    a->set_source(sink);
    // a->print_bend();
    // twopin_netlist_L.push_back(L_net);
}

void Router::two_pin_net_Z_routing(Pin*a , Pin*b){
    
    Cell* c_a = _placement->getCell(a->getcellId());
    Cell* c_b = _placement->getCell(b->getcellId());
    int a_x = c_a->getx();
    int a_y = c_a->gety();
    int a_z = a->get_layer();
    int b_x = c_b->getx();
    int b_y = c_b->gety();
    int b_z = b->get_layer();

    int z = 0;
    branch* branch_a = new branch(a_x,a_y,a_z);
    branch* branch_b = new branch(b_x,b_y,b_z);
    two_pin_net* Z_net = new two_pin_net(branch_a,branch_b);
    
    Bend* sink = new Bend(a_x,a_y,z); 
    Bend* Z_bend;
    Z_bend = Z_routing(a_x,b_x,a_y,b_y,z);

    Z_net->set_source ( sink );
    sink->set_next(Z_bend);
    Z_bend->set_prev(sink);

    twopin_netlist_Z.push_back(Z_net);
}

Bend* Router::Z_routing(size_t x1, size_t x2, size_t y1 ,size_t y2 ,size_t z){

    double z_cost_h ,z_cost_v ;
    Bend* Z_bend_h;
    Bend* Z_bend_v;
    Bend* sink = new Bend(x1,y1,z); 
    // Z_net->set_source ( new bend(x1,y1,z) );
    z_cost_h = Z_route_2D_H(x1,x2,y1,y2,z, Z_bend_h);
    z_cost_v = Z_route_2D_V(x1,x2,y1,y2,z, Z_bend_v);
    bool z_v = true;
    if(z_cost_h < z_cost_v){
        z_v = false;
    }
    else if(z_cost_h  == z_cost_v ){
        if(rand()%2 == 0){
            z_v = false;
        }
    }
    cout<<"h Z " << z_cost_h<<endl;
    Z_bend_h->print();
    cout<<"v Z " << z_cost_v<<endl;
    Z_bend_v->print();
    
    if( z_v == true ) { // choose two vertical route one horizontal route
        sink->set_next(Z_bend_v);
        Z_bend_v->set_prev(sink);
        Bend* temp = Z_bend_h->get_next();
        delete temp;
        delete Z_bend_h;
        
        return Z_bend_v;
    }
    else{
        sink->set_next(Z_bend_h);
        Z_bend_h->set_prev(sink);
        Bend* temp = Z_bend_v->get_next();
        delete temp;
        delete Z_bend_v;

        return Z_bend_h;
    } 
    
}

double Router::Z_route_2D_H(size_t x1, size_t x2, size_t y1, size_t y2 , size_t z , Bend*& Z_bend){
    double best_cost = 1e10;
    Bend* Z_bend_n;
    
    //asssume x1 < x2
    bool inv = false;
    if( x1 > x2 ){
        swap(x1,x2);
        swap(y1,y2);
        inv = true;
    }

    double best_x = 0 ;

    for (size_t x = x1+1 ; x <= x2; x++){
        double cost = H_route(x1,x,y1,z) + V_route(x,y1,y2,z) + H_route(x,x2,y2,z);
        if( cost < best_cost ){
            // delete Z_bend;
            // Z_bend = new bend(x,y1,z);
            best_cost = cost;
            best_x = x;
        }
    }
    if( inv == false){
        Z_bend = new Bend(best_x,y1,z);
        Z_bend_n = new Bend(best_x,y2,z);
        Z_bend->set_next( Z_bend_n );
        Z_bend_n->set_prev(Z_bend); 
    }
    else{
        if( best_x != x2){
            Z_bend = new Bend(best_x,y2,z);
            Z_bend_n = new Bend(best_x,y1,z);
            Z_bend->set_next( Z_bend_n );
            Z_bend_n->set_prev(Z_bend);
        }
        else    // L routing only one bend exist
            Z_bend = new Bend(best_x,y1,z);
    }
    return best_cost;
}

double Router::Z_route_2D_V(size_t x1, size_t x2, size_t y1, size_t y2 , size_t z , Bend*& Z_bend){
    double best_cost = 1e10;
    Bend* Z_bend_n;
    //asssume y1 < y2
    bool inv = false;
    if(y1 > y2){
        swap(y1,y2);
        swap(x1,x2);
        inv = true;
    }

    double best_y = 0;
    for (size_t y = y1+1 ; y <= y2; y++){
        double cost = V_route(x1,y1,y,z) + H_route(x1,x2,y,z) + V_route(x2,y,y2,z);
        if( cost < best_cost ){
            // delete Z_bend;
            // Z_bend = new bend(x1,y,z);
            best_cost = cost;
            best_y = y;
        }
    }

    if( inv == false){
        Z_bend = new Bend(x1,best_y,z);
        Z_bend_n = new Bend(x2,best_y,z);
        Z_bend->set_next( Z_bend_n );
        Z_bend_n->set_prev(Z_bend); 
    }
    else{
        if(best_y != y2 ){
            Z_bend = new Bend(x2,best_y,z);
            Z_bend_n = new Bend(x1,best_y,z);
            Z_bend->set_next( Z_bend_n );
            Z_bend_n->set_prev(Z_bend);
        }
        else {  //L routing only one bend exist
            Z_bend = new Bend(x1,best_y,z);
        }
    }
    return best_cost;
     
}

Bend* Router::L_route_2D(size_t x1, size_t x2, size_t y1, size_t y2 , size_t z){
    double L1_cost = 0;
    double L2_cost = 0;
    // int z = 0 ; 
    L1_cost = V_route(x1,y1,y2,z) + H_route(x1,x2,y2,z);
    L2_cost = H_route(x1,x2,y1,z) + V_route(x2,y1,y2,z);

    Bend* L_bend = new Bend(x1,y2,z) ; // L1_cost > L2_cost
    if(L1_cost < L2_cost ){
        delete L_bend;
        L_bend = new Bend(x2,y1,z);
    }
    else if(L1_cost == L2_cost){
        if( rand()%2 == 0){
            delete L_bend;
            L_bend = new Bend(x2,y1,z);
        } 
    }
    if( L_bend->_x == x2 && L_bend->_y == y2){
        return L_bend;
    }
    Bend* target = new Bend(x2,y2,z);
    L_bend->set_next( target );
    target->set_prev( L_bend );

    return L_bend;
 
}

double Router::V_route(size_t x , size_t y1, size_t y2 , size_t z ){
//y1 to y2 

    return V_route_edge( x ,  y1,  y2 ,  z );
    return V_route_grid( x ,  y1,  y2 ,  z );
}

double Router::H_route(size_t x1 , size_t x2, size_t y , size_t z ){
//x1 to x2
    
    return H_route_edge( x1 ,  x2,  y ,  z );
    return H_route_grid( x1 ,  x2,  y ,  z );
}

double Router::V_route_edge(size_t x , size_t y1, size_t y2 , size_t z ){
// y1 to y2 computed by edge 
    double h_cost = 0;
    // assume y1 < y2 
    if(y1 > y2)
        swap(y1,y2);
    for(size_t y = y1; y < y2 ; y++){
        h_cost += (*cost_col_map)(x,y,z);
        // cout<<x<<" "<<y<<" "<<z<<endl;
        // cout<<h_cost<<endl;
    }
    // cout<<"horizontal cost"<<h_cost<<endl;   
    return h_cost;
}

void Router::expansion(){
    double expand_x = 0 ,expand_y = 0;
    int i;
    for(i = 0 ; i < x_expand_factor.size() ; i++){
        x_expand_result[i] = expand_x;
        expand_x += x_expand_factor[i];
        y_expand_result[i] = expand_y;
        expand_y += y_expand_factor[i];
    }
    x_expand_result[i] = expand_x;
    y_expand_result[i] = expand_y;
}

double Router::V_route_grid(size_t x , size_t y1, size_t y2 , size_t z ){
//y1 to y2 compute_by grid
    double h_cost = 0;
    // assume y1 < y2 
    if(y1 > y2)
        swap(y1,y2);
    for(size_t y = y1; y < y2 ; y++){
        h_cost += (*cost_row_map)(x,y,z);
        // cout<<x<<" "<<y<<" "<<z<<endl;
        // cout<<h_cost<<endl;
    }
    // cout<<"horizontal cost"<<h_cost<<endl;
    return h_cost;
}

double Router::H_route_edge(size_t x1 , size_t x2, size_t y , size_t z ){
//x1 to x2 compute_by edge
    double v_cost = 0;
    // assume x1 < x2
    if(x1 > x2)
        swap(x1,x2); 
        // cout<<"x2 is "<<x2<<endl;
        // cout<<"x1 is "<<x1<<endl;
    for(size_t x = x1; x < x2 ; x++){
        // cout<<x<<" "<<y<<" "<<z<<endl;
        v_cost += (*cost_row_map)(x,y,z);
    }
    // cout<<"vertival cost"<<v_cost<<endl;
    return v_cost;
}

double Router::H_route_grid(size_t x1 , size_t x2, size_t y , size_t z ){
//x1 to x2 compute_by grid
    double v_cost = 0;
    // assume x1 < x2
    if(x1 > x2)
        swap(x1,x2); 
    for(size_t x = x1; x < x2 ; x++){
        // cout<<x<<" "<<y<<" "<<z<<endl;
        v_cost += (*cost_col_map)(x,y,z);
    }
    // cout<<"vertival cost"<<v_cost<<endl;
    return v_cost;
}

void Router::maze_routing(){

}

void Router::A_star_search_routing(){
    update_cost_map();
    for(size_t i = 0 ; i < two_pin_netlist.size(); i++){
        for(size_t j = 0; j < two_pin_netlist[i].size();j++){
            int s_x = two_pin_netlist[i][j].b_source->_x;
            int s_y = two_pin_netlist[i][j].b_source->_y;
            int t_x = two_pin_netlist[i][j].b_target->_x;
            int t_y = two_pin_netlist[i][j].b_target->_y;
            
            if( abs(s_x-t_x) <= 0) continue;
            if( abs(s_y-t_y) <= 0) continue;
            Shortest_Path test( s_x, s_y, 0, t_x, t_y, 0, 1, cost_row_map, cost_col_map, demand_grid_map );
            cout<<"Dijkstra..."<<endl;
            test.Dijkstra();
            cout<<"Build the path..."<<endl;
            test.Build_the_path();
            Bend* result = test.source();
            Bend* old_bend = two_pin_netlist[i][j].get_source();

            cout<<"Exclude the demand"<<endl;
            Exclude_demand(old_bend,1);
            two_pin_netlist[i][j].set_source(result);
            cout<<"ADD demand..."<<endl;
            Add_demand(result,1);

        }   
    }
}

void Router::update_cost_map(){
    double h = 0.01;
    double k = 1;
    int width = _placement->_boundary_width;
    int height = _placement->_boundary_height;
    // int norm_y_factor = _placement->_leftBoundary;
    // cost_row_map->print_congestion();
    for(int x = 0 ; x < width-1  ; x++){
        for(int y = 0 ; y < height; y++){

            (*cost_row_map)(x,y,0) = 1 + h/(1+exp(-k * (*demand_row_map)(x,y,0) - (*supply_row_map)(x,y,0))) ;
        }
    }
    
    for(int x = 0 ; x < width ; x++){
        for(int y = 0 ; y < height-1 ; y++){
           (*cost_col_map)(x,y,0) = 1 + h/(1+exp(-k * (*demand_col_map)(x,y,0) - (*supply_col_map)(x,y,0))) ;
        }
    }
}

Tree Router::Flute_function(vector<double> a, vector<double> b){
    Tree flutetree;
    int flutewl;
    int d=a.size();
    double* x = &a[0];
    double* y = &b[0];

    flutetree = flute(d, x, y, ACCURACY);
    // printf("FLUTE wirelength = %lf\n", flutetree.length);
    // printtree(flutetree);

    flutewl = flute_wl(d, x, y, ACCURACY);
    // printf("FLUTE wirelength (without RSMT construction) = %lf\n", flutewl);

    return flutetree;
}

void Router::construct_two_pin_net( Net* a, int idx){
    int pin_num = a->getPin_num();
    vector<double> x,y;
    x.resize(pin_num);
    y.resize(pin_num);
    int norm_x_factor = _placement->_leftBoundary;
    int norm_y_factor = _placement->_bottomBoundary;
    for(size_t j = 0 ; j < pin_num ; j++){
        Cell* c = _placement->getCell( a->getPin(j)->getcellId());
        x[j] = c->getx() - norm_x_factor;
        y[j] = c->gety() - norm_y_factor;
    }
    // if( pin_num <= 2){
    //     int z = 0;
    //     branch<int>* b1 = new branch<int>( x[0], y[0], z , 1); 
    //     branch<int>* b2 = new branch<int>( x[1], y[1], z , 0);
    //     two_pin_net<int> tp_net(b1,b2);
    //     two_pin_netlist[idx].push_back(tp_net);
    //     return;
    // }
    cout<<"Net: "<<a->getName()<<endl;
    Tree t = Flute_function(x,y);    
    int i;
    double p1_x,p1_y,p2_x,p2_y;
    two_pin_net tp_net;
    map<pair<int,int>,int> dummy_node_map;
    map<pair<int,int>,int>::iterator it;
    vector<int> dummy_node;
    dummy_node.resize(2*t.deg-2);
    int node_num = 0;
    for (i=0; i<t.deg; i++){
        p1_x = t.branch[i].x;
        p1_y = t.branch[i].y;
        int n = t.branch[i].n;
        double  z = a->getPin(i)->get_layer();
        branch* b = new branch( p1_x, p1_y, z, i, n); 
        // cout<<"index: "<<i<<" x: "<<p1_x<<" y: "<<p1_y<<endl;
        
        it = dummy_node_map.find( make_pair(p1_x,p1_y) );
        if( it == dummy_node_map.end()){
            // cout<<"new node: "<<i<<endl<<endl;
            dummy_node_map[make_pair(p1_x,p1_y)] = node_num;
            dummy_node[i] = node_num;
            b->_id = node_num;
            node_num++;
            branch_of_netlist[idx].push_back(b);
        }
        else{
            // cout<<"old node: "<<it->second<<endl<<endl;
            dummy_node[i] = it->second;
            // cout<<dummy_node[i]<<endl<<endl;
            b->_id = node_num;
            node_num++;
            branch_of_netlist[idx].push_back(b);
        }
    }
    for (i=t.deg; i<2*t.deg-2; i++){
        p1_x = t.branch[i].x;
        p1_y = t.branch[i].y;
        int n = t.branch[i].n;
        double z = -1;
        if(i < t.deg)
            z = a->getPin(i)->get_layer();
        branch* b = new branch( p1_x, p1_y, z, i, n); 
        // cout<<"index: "<<i<<" x: "<<p1_x<<" y: "<<p1_y<<endl;
        
        it = dummy_node_map.find( make_pair(p1_x,p1_y) );
        if( it == dummy_node_map.end()){
            // cout<<"new node: "<<i<<endl<<endl;
            dummy_node_map[make_pair(p1_x,p1_y)] = node_num;
            dummy_node[i] = node_num;
            b->_id = node_num;
            node_num++;
            branch_of_netlist[idx].push_back(b);
        }
        else{
            // cout<<"old node: "<<it->second<<endl<<endl;
            dummy_node[i] = it->second;
            // cout<<dummy_node[i]<<endl<<endl;
            // branch_of_netlist[idx].push_back(b);
        }
    }
    // cout<<"size of dummy_node: "<<dummy_node_map.size()<<endl;
    for (i= 0; i<2*t.deg-2; i++){
        p1_x = t.branch[i].x;
        p1_y = t.branch[i].y;
        int n = t.branch[i].n;
        // cout<<*branch_of_netlist[idx][dummy_node[i]]<<endl;
        if( n == i) continue;
        p2_x = t.branch[n].x;
        p2_y = t.branch[n].y;
        if( (p1_x == p2_x) && (p1_y == p2_y) ) continue;
        double z = 0;
        // i = dummy_node[i];
        n = dummy_node[n];
        int dummy_i = dummy_node[i];
        // if( n == i) continue;
        // cout<<"index: "<<i<<" corresponding inex: "<<dummy_node[i]<<" neighbor: "<<n<<endl;
        if( branch_of_netlist[idx][dummy_node[i]]->_id != dummy_node[i] ) continue;
        // cout<<"size of branch: "<<branch_of_netlist[idx].size()<<endl;
        // cout<<"index: "<<i<<" neighbor: "<<n<<endl;
        branch* b1 = branch_of_netlist[idx][dummy_i];
        b1->_n = n;
        branch* b2 = branch_of_netlist[idx][n];
        b2->_n = i;
        two_pin_net tp_net(b1,b2);
        // cout<<"2pin net constructed"<<endl;
        two_pin_netlist[idx].push_back(tp_net);
    }
    // cout<<"total 2pin net is constructed"<<endl<<endl;
}

void Router::construct_two_pin_net_with_expansion(Net*a ,int idx){
    // int pin_num = a->getPin_num();
    // vector<double> x,y;
    // x.resize(pin_num);
    // y.resize(pin_num);
    // expansion();
    // for(size_t j = 0 ; j < pin_num ; j++){
    //     Cell* c = _placement->getCell( a->getPin(j)->getcellId());
    //     x[j] = c->getx();
    //     x[j] *= x_expand_result[x[j]];
    //     y[j] = c->gety(); 
    //     y[j] *= y_expand_result[y[j]];
    // }
    // // if( pin_num <= 2){
    // //     double z = 0;
    // //     branch<int>* b1 = new branch<int>( x[0], y[0], z ); 
    // //     branch<int>* b2 = new branch<int>( x[1], y[1], z );
    // //     two_pin_net<int> tp_net(b1,b2);
    // //     two_pin_netlist[idx].push_back(tp_net);
    // //     return;
    // // }
    // Tree t = Flute_function(x,y);    
    // int i;
    // double p1_x,p1_y,p2_x,p2_y;
    // two_pin_net<int> tp_net;
    // for (i=0; i<t.deg; i++){
    //     p1_x = t.branch[i].x;
    //     p1_x = find_expand_x_position(p1_x);
    //     p1_y = t.branch[i].y;
    //     p1_y = find_expand_y_position(p1_y);
    //     int n = t.branch[i].n;
    //     if( n == i) continue;
    //     p2_x = t.branch[n].x;
    //     p2_x = find_expand_x_position(p2_x);
    //     p2_y = t.branch[n].y;
    //     p2_y = find_expand_y_position(p2_y);
    //     if( p1_x == p2_x && p2_x == p2_y) continue;
    //     int z = 0;
    //     branch<int>* b1 = new branch<int>( p1_x, p1_y, z ); 
    //     branch<int>* b2 = new branch<int>( p2_x, p2_y, z );
    //     two_pin_net<int> tp_net(b1,b2);
    //     two_pin_netlist[idx].push_back(tp_net);
    // }
    

}

void Router::construct_total_two_pin_net(bool expand){
    two_pin_netlist.clear();
    two_pin_netlist.resize(_placement->_numNets);
    branch_of_netlist.resize(_placement->_numNets);
    segment_of_netlist.resize(_placement->_numNets);
    readLUT();
    if(expand == false){
        for(int i = 0 ; i < _placement->_numNets ; i++){
            construct_two_pin_net( _placement->_netArray[i], i );
        }
    }
    else{
        for(int i = 0 ; i < _placement->_numNets ; i++){
            construct_two_pin_net_with_expansion( _placement->_netArray[i], i );
        }
    }
}

void Router::update_x_expansion(int idx , double f){
    x_expand_factor[idx] = f;
}

int Router::find_expand_x_position(double x){
    int left = 0, right = x_expand_result.size() - 1;
    while (left <= right)
    {
        int middle = (right + left) / 2;

        if (x_expand_result[middle] == x)
            return middle;

        if (x_expand_result[middle] > x)
            right = middle - 1;
        else
            left = middle + 1;
    }
}

void Router::update_y_expansion(int idx , double f){
    y_expand_factor[idx] = f;
}

int Router::find_expand_y_position(double y){
    int left = 0, right = y_expand_result.size() - 1;
    while (left <= right)
    {
        int middle = (right + left) / 2;

        if (y_expand_result[middle] == y)
            return middle;

        if (y_expand_result[middle] > y)
            right = middle - 1;
        else
            left = middle + 1;
    }
}

void Router::Add_demand(Bend* start, double f){        //2D
    Bend* trav = start;                 //traverse
    Bend* n_trav = trav->get_next();    //next traverse
    while(n_trav != NULL){
        if(trav->_x == n_trav->_x){         //Vertical
            Add_demand_V(trav->_x, trav->_y, n_trav->_y, 0, f);
        }
        else if(trav->_y == n_trav->_y){    //Horizontal
            Add_demand_H(trav->_x, n_trav->_x , trav->_y, 0, f);
        }
        trav = n_trav;
        n_trav = trav->get_next();
    }
}

void Router::Add_demand_3D(Bend* s, double f = 1){
    Bend* trav = s;                 //traverse
    Bend* n_trav = trav->get_next();    //next traverse
    while(n_trav != NULL){
        if(trav->_x != n_trav->_x){         //Horizontal
            Add_demand_3D_H(trav->_x, n_trav->_x, trav->_y, trav->_z, f);
        }
        else if(trav->_y != n_trav->_y){    //Vertical
            Add_demand_3D_V(trav->_x, trav->_y , n_trav->_y, trav->_z, f);
        }
        else if(trav->_z != n_trav->_z){    //Z_direction
            Add_demand_3D_Z(trav->_x, trav->_y , trav->_z, n_trav->_z, f);
        }
        trav = n_trav;
        n_trav = trav->get_next();
    }
}

void Router::Add_demand_H( int x1, int x2, int y, int z, double f){
    if(x1 > x2) 
        swap(x1,x2); //make x1 < x2
    for(int x = x1; x < x2; x++){
        (*demand_grid_map)(x,y,z) += f;
    }
}

void Router::Add_demand_V( int x, int y1, int y2, int z, double f){
    if(y1 > y2) 
        swap(y1,y2); //make y1 < y2
    for(int y = y1; y < y2; y++){
        (*demand_col_map)(x,y,z) += f;
    }
}

void Router::Add_demand_3D_H(int x1, int x2, int y, int z, double f ){
    if(x1 > x2) 
        swap(x1,x2); //make x1 < x2
    for(int x = x1; x < x2; x++){
        (*demand_grid_map)(x,y,z) += f;
    }
}

void Router::Add_demand_3D_V(int x, int y1, int y2, int z, double f ){
    if(y1 > y2) 
        swap(y1,y2); //make y1 < y2
    for(int y = y1; y < y2; y++){
        (*demand_grid_map)(x,y,z) += f;
    }
}

void Router::Add_demand_3D_Z(int x, int y, int z1, int z2, double f ){
    if(z1 > z2) 
        swap(z1,z2); //make z1 < z2
    for(int z = z1; z < z2; z++){
        (*demand_grid_map)(x,y,z) += f;
    }
}

void Router::Exclude_demand(Bend* start, double f){        //2D and we need remove the bend
    Bend* trav = start;                 //traverse
    Bend* n_trav = trav->get_next();    //next traverse
    while(n_trav != NULL){
        if(trav->_x == n_trav->_x){   //Vertical
            Exclude_demand_V(trav->_x, trav->_y, n_trav->_y, 0, f);
        }
        else if(trav->_y == n_trav->_y){    //Horizontal
            Exclude_demand_H(trav->_x, n_trav->_x , trav->_y, 0, f);
        }
        delete trav;
        trav = n_trav;
        n_trav = trav->get_next();
    }
}

void Router::Exclude_demand_H( int x1, int x2, int y, int z, double f){
    if(x1 > x2) 
        swap(x1,x2); //make x1 < x2
    for(int x = x1; x < x2; x++){
        (*demand_row_map)(x,y,z) -= f;
    }
}

void Router::Exclude_demand_V( int x, int y1, int y2, int z, double f){
    if(y1 > y2) 
        swap(y1,y2); //make y1 < y2
    for(int y = y1; y < y2; y++){
        (*demand_col_map)(x,y,z) -= f;
    }
}

void Router::construct_congestion_map(){
    construct_total_two_pin_net(false);
    bool first_round = true;
    // return;
    // add demand
    for(int i = 0; i < two_pin_netlist.size() ; i++){
        for(int j = 0 ; j < two_pin_netlist[i].size() ; j++){
            two_pin_net_Both_L_routing( &two_pin_netlist[i][j] , first_round);
        }
    }
    // cout<<"demand added"<<endl;
    first_round = false;
    for(int i = 0; i < two_pin_netlist.size() ; i++){
        for(int j = 0 ; j < two_pin_netlist[i].size() ; j++){
            two_pin_net_Both_L_routing( &two_pin_netlist[i][j] , first_round);
            two_pin_net_L_routing( &two_pin_netlist[i][j] );
        }
    }
}

void Router::projection_to_2D(){
    int width =  _placement->_boundary_width;
    int height = _placement->_boundary_height;
    int layer = _placement->_numLayers;
    for(int x = 0 ; x < width; x++){
        for(int y = 0 ; y < height; y++){
            double supply = 0;
            double demand = 0;
            for(int z = 0 ; z < layer; z++){
                supply += (*supply_grid_map)(x,y,z);
                demand += (*demand_grid_map)(x,y,z);
            }
            (*supply_grid_2Dmap)(x,y,0) = supply-demand;
            // (*demand_grid_2Dmap)(x,y,0) = demand;
        }
    }
}

void Router::supply_from_grid_to_edge(){
//2D
    int width =  _placement->_boundary_width;
    int height = _placement->_boundary_height;

    // row map
    for(int x = 0 ; x < width-1; x++){
        for(int y = 0 ; y < height; y++){
            double supply_1 = (*supply_grid_2Dmap)(x,y,0);
            double supply_2 = (*supply_grid_2Dmap)(x,y,0);
            (*supply_row_map)(x,y,0) = min(supply_1,supply_2);
        }
    }
    // col map
    for(int x = 0 ; x < width; x++){
        for(int y = 0 ; y < height-1; y++){
            double supply_1 = (*supply_grid_2Dmap)(x,y,0);
            double supply_2 = (*supply_grid_2Dmap)(x,y,0);
            (*supply_col_map)(x,y,0) = min(supply_1,supply_2);
        }
    }
}

void Router::layer_assignment(){

    compute_total_net_length();
    update_distance_of_branch();
    // vector<Net*>
    vector<size_t> sorted_idx_of_net = sort_indexes(_placement -> _netArray);
    for(int i = 0; i < sorted_idx_of_net.size(); i++){
        cout<<"layer assignment : "<<sorted_idx_of_net[i]<<endl;
        layer_assignment_one_net(sorted_idx_of_net[i]);
        
    }
    // for(int i = 0 ; i < two_pin_netlist.size(); i++){
    //     cout<<"layer assignment : "<<i<<endl;
    //     layer_assignment_one_net(i);
    // }
    
}

void Router::layer_assignment_one_net(int net_idx){
    
    int width = _placement->_boundary_width;
    int height = _placement->_boundary_height;

    pair<int,int> initial_pair = make_pair(-1,-1);
    layer_range = Two_Dimension_map<pair<int,int>>(width,height,initial_pair);
    vector<vector<Segment>> total_segment;    
    int pin_num = _placement -> _netArray[net_idx] -> getPin_num();
    int total_number_segment = 0;
    for(int i = 0 ; i < two_pin_netlist[net_idx].size() ; i++){
        // two_pin_netlist[net_idx][i].print_bend();
        branch* s = two_pin_netlist[net_idx][i].b_source;
        branch* t = two_pin_netlist[net_idx][i].b_target;
        bool s_is_pin = ( s->_id < pin_num ) ? true : false;
        bool t_is_pin = ( t->_id < pin_num ) ? true : false;
        int s_to_pin = s->dist_to_pin;
        int t_to_pin = t->dist_to_pin;  
        int seg_len = two_pin_netlist[net_idx][i].segment_length();
        int d_of_two_branch = abs( s_to_pin - t_to_pin );
        total_number_segment += seg_len;
        int max_dist_to_pin = seg_len + min(s_to_pin,t_to_pin);
        cout<<endl<<"================"<<endl;
        cout<<"max dist to pin: "<<max_dist_to_pin<<endl;
        cout<<endl<<"idx " <<i<<" segment len: "<<seg_len<<endl;
        cout<<"dist to pin: s: "<<s_to_pin<<" t: "<<t_to_pin<<endl;
        if( total_segment.capacity() < max_dist_to_pin ) total_segment.resize(max_dist_to_pin);
        Bend* travel = two_pin_netlist[net_idx][i].source;
        int n = 0;
        if( s_to_pin <= t_to_pin){
            int d = s_to_pin ;
            int increase_len = min ( seg_len , (seg_len - d_of_two_branch)/2 + d_of_two_branch ); 
            
            for ( ; n < increase_len ; n++, d++){
                Bend* next_bend = travel->get_next();
                total_segment[d].push_back( Segment(travel, next_bend) );
                travel = next_bend;
                // cout<<"n is "<<n<<" d is "<<d<<endl;
                assert(travel != NULL);
            }
            if ( n < seg_len && ( seg_len - d_of_two_branch) % 2 == 1 ){
                Bend* next_bend = travel->get_next();
                total_segment[d].push_back( Segment(travel, next_bend) );
                travel = next_bend;
                // cout<<"n is "<<n<<" d is "<<d<<endl;
                n++;
                d--;
                assert(travel != NULL);
            }
            for ( ; n < seg_len ; n++ ,d--){
                Bend* next_bend = travel->get_next();
                total_segment[d].push_back( Segment(travel, next_bend) );
                travel = next_bend;
                assert(travel != NULL);
                // cout<<"n is "<<n<<" d is "<<d<<endl;
            }
        }
        else if( s_to_pin > t_to_pin){
            cout<<"================="<<endl;
            cout<<"back from target"<<endl;
            int d = t_to_pin ;
            int decrease_len = min ( seg_len , (seg_len - d_of_two_branch)/2 + d_of_two_branch ); 
            // if( decrease_len > seg_len){
                
            // }
            int increase_len = seg_len - decrease_len;
            // cout<<"increase length: "<<increase_len<<endl;
            for ( ; n < increase_len ; n++, d++){
                Bend* next_bend = travel->get_next();
                total_segment[d].push_back( Segment(travel, next_bend) );
                travel = next_bend;
                // cout<<"n is "<<n<<" d is "<<d<<endl;
                assert(travel != NULL);
            }
            if ( n < seg_len && ( seg_len - d_of_two_branch) % 2 == 1 ){
                Bend* next_bend = travel->get_next();
                total_segment[d].push_back( Segment(travel, next_bend) );
                travel = next_bend;
                // cout<<"n is "<<n<<" d is "<<d<<endl;
                n++;
                d--;
                assert(travel != NULL);
            }
            for ( ; n < seg_len ; n++, d--){
                Bend* next_bend = travel->get_next();
                total_segment[d].push_back( Segment(travel, next_bend) );
                travel = next_bend;
                // cout<<"decreasing... n is "<<n<<" d is "<<d<<endl;
                assert(travel != NULL);
            }
        }        
        // cout<<"n = "<< n << " seg_length = "<< seg_len << " half seg_length = "<< seg_len/2 <<endl;
        // cout<<"total segment: capacity: "<<total_segment.capacity()<<" size "<<total_segment.size()<<endl;
        // cout<<"d = "<< d <<endl; 
    }
    // Bend* source = new Bend(0,0,4);
    // Bend* target = new Bend(0,4,0);
    bool Horizontal = true;
    int min_layer = _placement -> _netArray[net_idx]->getMinLayer();
    // int net_idx = 0;
    cout<<"layer assignment about segement Net: "<<net_idx<<endl;
    cout<<"total segment: "<< total_number_segment <<endl;
    for (int i = 0; i < total_segment.size(); i++){
        cout<<"the i is "<< i << " size "<< total_segment[i].size()<<endl;
        for( int j = 0; j < total_segment[i].size(); j++){
            Bend* source = total_segment[i][j].source;
            Bend* target = total_segment[i][j].target;
            if(source == NULL){
                total_segment[i][j].print();
            }
            assert(source != NULL); 
            assert(target != NULL);
            layer_assignment_straight_line(source, target, min_layer,net_idx);
        }
    }
}

void Router::compute_total_net_length(){
    for(size_t i = 0 ; i < two_pin_netlist.size(); i++){
        compute_one_net_length( i );
    }
}

void Router::compute_one_net_length(int idx){
    int length = 0;
    for(size_t i = 0; i < two_pin_netlist[idx].size(); i++){
        length += two_pin_netlist[idx][i].length();
    }
    cout<<"Net: "<<idx<<" length: "<<length<<endl;
    _placement -> _netArray[idx]->set_2D_len( length );
    
}

void Router::update_distance_of_branch(){
    cout<<"update distance of branch"<<endl;
    for(size_t i = 0 ; i < two_pin_netlist.size(); i++){
        update_distance_of_branch_in_one_net( i );
    }
}

void Router::update_distance_of_branch_in_one_net(int idx){
    // not sure about the order of two pin net can let all the branch assign correct distance
    int pin_num = _placement -> _netArray[idx] -> getPin_num();
    vector<int> non_traval;
    for(int i = 0 ; i < two_pin_netlist[idx].size() ; i++){
        branch* s = two_pin_netlist[idx][i].b_source;
        branch* t = two_pin_netlist[idx][i].b_target;
        bool s_is_pin = ( s->_id < pin_num ) ? true : false;
        bool t_is_pin = ( t->_id < pin_num ) ? true : false;
        int seg_len = two_pin_netlist[idx][i].segment_length();
        if(s_is_pin == true){
            s->dist_to_pin = 0;
            if(t_is_pin == false)
                t->dist_to_pin = min( t -> dist_to_pin , seg_len);
            else
                t->dist_to_pin = 0;
                
        }
        else if(t_is_pin == true){
            t->dist_to_pin = 0;
            s->dist_to_pin = min( s -> dist_to_pin , seg_len);
        }
        else {
            if( t->dist_to_pin != INT_MAX ){
                s->dist_to_pin = min( s->dist_to_pin , t->dist_to_pin + seg_len);
            }
            else if ( s->dist_to_pin != INT_MAX ){
                t->dist_to_pin = min( t->dist_to_pin , s->dist_to_pin + seg_len);
            }
            else {
                non_traval.push_back(i);
            }
        }
        
        // cout<<"idx " <<i<<"segment len: "<<seg_len<<endl; 
    }
    // cout << "NON traval "<< non_traval.size()<<endl;
    if( non_traval.size() != 0){
        cerr<<"exit"<<endl;
    }
}

void Router::layer_assignment_two_pin_net(int net_id, two_pin_net a){
    
    // cout<<"source's neighbor: "<<a.b_source -> _n<<endl;
    int branch_idx = branch_of_netlist[net_id].size();
    // a.print_bend();
    int source_id = a.b_source -> _id; 
    int target_id = a.b_target -> _id;
    if( source_id < _placement -> _netArray[net_id] -> getPin_num())
        a.b_source ->_z = _placement -> _netArray[net_id] -> getPin(source_id) -> get_layer();
    // if( *(two_pin_netlist[idx][i].b_source) == *(two_pin_netlist[idx][i].b_target))
    //     continue;
    Bend* temp = a.get_source();
    Bend* n_temp = temp -> get_next();

    int min_layer = _placement -> _netArray[net_id] -> getMinLayer();
    int routing_layer = layer_assignment_straight_line_old_method(temp, n_temp, min_layer);
    // cout<<"routing layer: "<<routing_layer<<endl;

    if( source_id < _placement -> _netArray[net_id] -> getPin_num()){
        // int target_idx = two_pin_netlist[idx][i] -> b_target ->_n;
        int source_layer = _placement -> _netArray[net_id] -> getPin(source_id) -> get_layer();
        int first_idx;
        // cout<<"source_id: "<<source_id<<endl;
        // cout<<"source layer: "<<source_layer<<endl;
        // a.b_source ->_z = source_layer;
        branch* trav = a.b_source;
        // cout<<"source in two pin net: "<<trav->_x<<" "<<trav->_y<<" "<<trav->_z<<" "<<endl;
        // cout<<"neighbor: "<<a.b_source -> _n<<endl;

        if(trav->_n == target_id){  // first touch source
            trav->_n = source_id;
        }
        // else(if)
        // cout<<"branch_idx: "<<branch_idx<<endl;
        if( source_layer != routing_layer ){
            int x = trav -> _x;
            int y = trav -> _y;
            branch* n_branch = new branch( x,y,routing_layer,branch_idx,source_id);
            // trav->_n = branch_idx;
            // trav->_n = source_id;
            branch_of_netlist[net_id].push_back(n_branch);
            // branch<int> * trav = n_branch;
            // cout<<"branch_idx: "<<branch_idx<<" "<<n_branch->_x<<" "<<n_branch->_y<<" "<<n_branch->_z<<" "<<endl;
            branch_idx++;
            
            x = n_temp -> _x;
            y = n_temp -> _y;
            
            int target_x = a.b_target -> _x;
            int target_y = a.b_target -> _y;
            
            // if( x == target_x && y == target_y){
            //     a.b_target->_z = routing_layer;
                
            // }
            branch* n2_branch = new branch( x,y,routing_layer,branch_idx,branch_idx-1);
            branch_of_netlist[net_id].push_back(n2_branch);
            // trav = n2_branch;
            // cout<<"branch2_idx: "<<branch_idx<<" "<<n2_branch->_x<<" "<<n2_branch->_y<<" "<<n2_branch->_z<<" "<<endl;
            branch_idx++;
        }
        else{                
            int x = n_temp -> _x;
            int y = n_temp -> _y;
            branch* n_branch = new branch( x,y,routing_layer,branch_idx,source_id);
            branch_of_netlist[net_id].push_back(n_branch);
            // cout<<"branch_idx: "<<branch_idx<<" "<<n_branch->_x<<" "<<n_branch->_y<<" "<<n_branch->_z<<" "<<endl;
            // trav->_n = branch_idx;
            branch_idx++;
            
        }
    }

    temp = n_temp;
    n_temp = temp->get_next(); 

    while(n_temp != NULL){  //until n_temp is target
        routing_layer = layer_assignment_straight_line_old_method(temp,n_temp,min_layer);
        // cout<<"routing layer: "<<routing_layer<<endl;
        int x = temp -> _x;
        int y = temp -> _y;
        branch* n_branch = new branch( x,y,routing_layer,branch_idx,branch_idx-1) ;
        branch_of_netlist[net_id].push_back(n_branch);
        branch_idx++;
        
        // branch<int> * trav = n_branch;
        // cout<<n_branch->_x<<" "<<n_branch->_y<<" "<<n_branch->_z<<" "<<endl;
        x = n_temp -> _x;
        y = n_temp -> _y;
        branch* n2_branch = new branch( x,y,routing_layer,branch_idx,branch_idx-1) ;
        branch_of_netlist[net_id].push_back(n2_branch);
        branch_idx++;
        temp = n_temp;
        n_temp = temp->get_next(); 
        // trav = n2_branch;
        // cout<<n2_branch->_x<<" "<<n2_branch->_y<<" "<<n2_branch->_z<<" "<<endl;
        
    }
    // cout<<"target_idx "<<target_id<<endl;
    int target_layer = a.b_target ->_z;
    // int target_layer;
    if( target_id < _placement -> _netArray[net_id] -> getPin_num()){
        target_layer = _placement -> _netArray[net_id] -> getPin(target_id) -> get_layer();
    }
    if( target_layer == -1){
        // target branch belongs to steiner node
        a.b_target ->_z = routing_layer;
    }
    else if( target_layer != routing_layer ){
        // int x = a.b_target -> _x;
        // int y = a.b_target -> _y;

        // branch<int>* n_branch = new branch<int>( x,y,routing_layer,branch_idx,target_idx);
        // branch_of_netlist[net_id].push_back(n_branch);
        // branch_idx++;
        // cout<<"target id: "<<a.b_target->_id<<" ("<<a.b_target->_x<<","<<a.b_target->_y<<","<<a.b_target->_z<<")"<<endl;
        // cout<<"neighbor: "<<a.b_target -> _n<<endl;
        a.b_target -> _n = (branch_idx-1);
    }
}

void Router::z_dirertion_layer_assignment(branch *a, branch *b){
    int a_x = a->_x;
    int a_y = a->_y;
    int a_z = a->_z;
    int b_x = b->_x;
    int b_y = b->_y;
    int b_z = b->_z;
    if( a_x != b_x) return;
    int x = a_x;
    if( a_y != b_y) return;
    int y = a_y;
    if(a_z > b_z)   //make a_z < b_z
        swap(a_z,b_z);
    
    for(int z = a_z + 1 ; z < b_z ;z++){
        if( (*supply_grid_map)(x,y,z) - (*demand_grid_map)(x,y,z) < 1){
            // congestion = true;
            // break;
            exit(-1);
        }
        (*demand_grid_map)(x,y,z)++;
    }
}

void Router::layer_assignment_straight_line(Bend* source, Bend* target, int min_layer , int net_idx = 0){
    cout<<endl;
    cout<<"layer assignment the straingt line"<<endl;
    assert(source != NULL);
    assert(target != NULL);
    int s_x = source->_x;
    int s_y = source->_y;
    int s_layer = source->_z;
    int t_x = target->_x;
    int t_y = target->_y;
    int t_z = target->_z;
    bool limit_range = (t_z == -1 ? 0 : 1);
    cout<<"s"<<s_x<<" "<<s_y<<endl;
    cout<<"t"<<t_x<<" "<<t_y<<endl;
    pair<int,int>& s_layer_range = layer_range(s_x,s_y);
    pair<int,int>& t_layer_range = layer_range(t_x,t_y);
    int width = _placement->_boundary_width;
    int height = _placement->_boundary_height;
    int layer_num = _placement->_numLayers;
    width = abs(s_x - t_x) + 1;
    height = abs(s_y - t_y) + 1;
    // width = abs(width) + 1;
    // height = abs(height) + 1;
    int left_bound = min(s_x,t_x);
    int lower_bound = min(s_y,t_y);
    cout<<"width: "<<width<<endl;
    cout<<"height: "<<height<<endl;
    Congestion via_grid(width,height,layer_num,INFINITY);
    // true means 2D , false means z direction
    // cout<<"origianl coluumn "<<via_grid(s_x,s_y,s_layer+2)<<endl;
    int code = 0;
    Via_Direction original_dir; 
    bool Horizontal = true;
    if(s_y != t_y){
        Horizontal = false;
    }
    cout<<"insertion"<<endl;
    if(Horizontal){
        assert(s_y == t_y);
        code = (s_x > t_x ) ? 3 : 2;
        original_dir = (s_x > t_x ) ? Pos_x : Neg_x;
    }
    else{
        assert(s_x == t_x);
        code = (s_y > t_y ) ? 1 : 0;
        original_dir = (s_y > t_y ) ? Pos_y : Neg_y;
    }
    if(min_layer == -1){
        min_layer = ( Horizontal ) ? 0 : 1;
    }
    else{        
        if( !Horizontal && min_layer % 2 == 0)
            ++min_layer;
        else if( Horizontal && min_layer % 2 == 1)
            ++min_layer;
    }
    // cout << "original dir" << original_dir <<endl;
    // cout << "Pos_x " << Pos_x<<endl;
    vector<vector<vector< Via_Direction >>> via_dir = vector<vector<vector<Via_Direction>>> ( width , vector<vector<Via_Direction>>(height , vector<Via_Direction>(layer_num,original_dir)) );
    int source_idx = ( Horizontal ) ? s_x : s_y;
    int target_idx = ( Horizontal ) ? t_x : t_y;
    int travel_idx = source_idx;        
    // (code % 2 == 0 ) ? (travel_idx++) : (travel_idx--);
    int x = s_x;
    int y = s_y;
    int norm_x, norm_y ;
    // (Horizontal) ? ( x = travel_idx ) : ( y = travel_idx );
    norm_x = x;
    norm_y = y;
    norm_x -=  left_bound;
    norm_y -=  lower_bound;
    int constraint_layer = (Horizontal) ? 0 : 1;
    int& lower_layer = s_layer_range.first;
    int& upper_layer = s_layer_range.second;
    cout<<"sourece layer"<<endl;
    cout<<"from "<<lower_layer<<" to "<<upper_layer<<endl;
    if( lower_layer == -1 ) {
        lower_layer = s_layer;
        upper_layer = s_layer;
        // via_grid(s_x-left_bound,s_y-lower_bound, s_layer) = 0;
    }
    else{
        if( s_layer < lower_layer ){
            lower_layer = s_layer;
        }
        else if(s_layer > upper_layer){
            upper_layer = s_layer;
        }
    }
    for(int z = lower_layer, j = 0 ; z >= 0; z--, j++){
        via_grid(s_x-left_bound,s_y-lower_bound, z) = j;
    }
    for(int z = lower_layer, j = 0 ; z < upper_layer; z++){
        via_grid(s_x-left_bound,s_y-lower_bound, z) = j;
    }
    for(int z = upper_layer, j = 0 ; z < layer_num; z++, j++){
        via_grid(s_x-left_bound,s_y-lower_bound, z) = j;
    }
    for(int z = 0; z < s_layer; z++){
        via_dir[s_x-left_bound][s_y-lower_bound][z] = Pos_z;
    }
    for(int z = s_layer + 1; z < layer_num; z++){
        via_dir[s_x-left_bound][s_y-lower_bound][z] = Neg_z;
    }
    via_dir[s_x-left_bound][s_y-lower_bound][s_layer] = End_point;

    // via_grid.print_congestion();

    // for(int i = 0; i < layer_set.size() - 1 ; i++){
    //     int lower_layer = layer_set[i];
    //     int upper_layer = layer_set[i+1];
    //     int range = upper_layer - lower_layer;
    //     via_grid(s_x-left_bound,s_y-lower_bound, lower_layer) = 0;
    //     cout<<endl;
    //     cout << "range is from "<<lower_layer<<" to "<<upper_layer<< " total range" << round(range/2)<<endl;
    //     cout<<endl;
    //     for(int j = 0; j < range / 2; j++,lower_layer++, upper_layer-- ){
    //         via_grid(s_x-left_bound,s_y-lower_bound, lower_layer) = j;
    //         via_grid(s_x-left_bound,s_y-lower_bound, upper_layer) = j;
    //     }
    //     // if(layer_set[i] % 2 == constraint_layer)
    //     //     via_grid(s_x-left_bound,s_y-lower_bound, layer_set[i]) = 0;
    //     // else {
    //     //     via_grid(s_x-left_bound,s_y-lower_bound, layer_set[i] + 1 ) = 1;
    //     //     via_grid(s_x-left_bound,s_y-lower_bound, layer_set[i] - 1 ) = 1;
    // }
    do{
        switch(code){
            //vertical edge
            case(0):{
                // poisitive edge
                y++; norm_y++; travel_idx++;
                break;
            }
            case(1):{
                // negative edge
                y--; norm_y--; travel_idx--;
                break;
            }
            //horizontal edge
            case(2):{
                // poistive edge
                x++; norm_x++; travel_idx++;
                break;
            }
            case(3):{
                //negative edge
                x--; norm_x--; travel_idx--;
                break;
            } 
        }
        // cout<<"current travel index: "<<travel_idx<<endl;
        // cout<<"norm x "<<norm_x<<" norm_y "<<norm_y<<endl;
        vector<Via_Grid_cost> via_grid_set;
        // (*supply_grid_map).print_congestion();
        // (*demand_grid_map).print_congestion();
        for(int z = min_layer, idx = 0 ; z < layer_num; z+=2, ++idx){
            int last_x = x; 
            int last_y = y;
            int last_norm_x = norm_x;
            int last_norm_y = norm_y;
            bool available = false;
            switch(code){
                //vertical edge
                case(0):{
                    // poisitive edge
                    --last_y;
                    --last_norm_y;
                    // via_grid(norm_x,norm_y,z) = via_grid(norm_x,norm_y-1,z); 
                    break;
                }
                case(1):{
                    // negative edge
                    ++last_y;
                    ++last_norm_y;
                    // via_grid(norm_x,norm_y,z) = via_grid(norm_x,norm_y+1,z);
                    break;
                }
                //horizontal edge
                case(2):{
                    // poistive edge
                    --last_x;
                    --last_norm_x;
                    // via_grid(norm_x,norm_y,z) = via_grid(norm_x-1,norm_y,z);
                    break;
                }
                case(3):{
                    //negative edge
                    ++last_x;
                    ++last_norm_x;
                    // via_grid(norm_x,norm_y,z) = via_grid(norm_x+1,norm_y,z); 
                    break;
                }
            }
            if( (*supply_grid_map)(x,y,z) > (*demand_grid_map)(x,y,z) ){
                // cout<<endl;
                // cout<<"last idx ("<<last_x<<","<<last_y<<","<<z<<")"<<endl;
                // cout<<"cur idx ("<<x<<","<<y<<","<<z<<")"<<endl;
                if( (*supply_grid_map)(last_x,last_y,z) > (*demand_grid_map)(last_x,last_y,z) ){
                    via_grid(norm_x,norm_y,z) = via_grid(last_norm_x, last_norm_y, z);
                    // cout<<"past from left"<<endl;
                }
                available = true;   
            }
            via_grid_set.push_back( Via_Grid_cost(&via_grid(norm_x,norm_y,z), z, idx, available ) );
            // cout<<"z = "<<z<<endl;
            // via_grid_set.back().print();
        }

        //update cost from neighbor
        
        priority_queue<Via_Grid_cost,vector<Via_Grid_cost>,Via_Grid_bigger_comp> sorted_VG_set( via_grid_set.begin(),via_grid_set.end() ) ;
        // sort( via_grid_set.begin(),via_grid_set.end(), Via_Grid_comparotor() ) ;
        Via_Grid_cost temp_grid = sorted_VG_set.top();
        // sorted_VG_set.pop();
        int visited_num = 1;
        while( visited_num != via_grid_set.size() ){
            // temp_grid.print();
            int cur_idx = temp_grid.idx;
            int cur_layer = temp_grid.layer;
            int upper_idx = cur_idx + 1;
            int lower_idx = cur_idx - 1;
            double min_via_cost = *(temp_grid.value) + 2;
            via_grid_set[cur_idx].visit = true;
            if( upper_idx < via_grid_set.size() ){
                int upper_z = cur_layer + 1 ;
                if( (*supply_grid_map)(x,y,upper_z) > (*demand_grid_map)(x,y,upper_z) ){
                    ++upper_z;
                    if( via_grid_set[upper_idx].visit == false && via_grid_set[upper_idx].avail)
                        if( *(via_grid_set[upper_idx].value) > min_via_cost ){
                            *(via_grid_set[upper_idx].value) =  min_via_cost;
                            // via_grid_set[upper_idx].visit = true;
                            sorted_VG_set.push(via_grid_set[upper_idx]);
                            via_dir[norm_x][norm_y][upper_z] = Neg_z;
                        }
                }
            }
            if( lower_idx >= 0 ){
                int lower_z = cur_layer - 1;
                if( (*supply_grid_map)(x,y,lower_z) > (*demand_grid_map)(x,y,lower_z) ){
                    --lower_z;
                    if( via_grid_set[lower_idx].visit == false && via_grid_set[lower_idx].avail)
                        if( *(via_grid_set[lower_idx].value) >  min_via_cost ){
                            *(via_grid_set[lower_idx].value) =  min_via_cost;
                            // via_grid_set[lower_idx].visit = true;
                            sorted_VG_set.push(via_grid_set[lower_idx]);
                            via_dir[norm_x][norm_y][lower_z] = Pos_z;
                        }
                }
            }

            // cout<<endl<<"update grid"<<endl;
            do{
                sorted_VG_set.pop();
                temp_grid = sorted_VG_set.top();
                cur_idx = temp_grid.idx;
                // temp_grid.print();
            }  
            while(via_grid_set[cur_idx].visit == true && via_grid_set[cur_idx].avail == true && !sorted_VG_set.empty());
            // cout<<"next grid"<<endl<<endl;
            visited_num++;
            // cout<<"visited_num "<<visited_num<<endl; 
        }
    }
    while(travel_idx != target_idx);
    // via_grid.print_congestion();

    // trace back
    cout<<endl;
    cout<<"========================"<<endl;
    cout<<"trace back"<<endl;
    lower_layer = (t_layer_range.first == -1) ? 0 : t_layer_range.first;
    upper_layer = (t_layer_range.second == -1) ? layer_num : t_layer_range.second;
    int min_cost_layer ;
    double min_cost = INFINITY;
    for(int z = 0; z < layer_num; z++){
        if(via_grid(t_x-left_bound, t_y-lower_bound, z) < min_cost){
            min_cost_layer = z;
            min_cost = via_grid(t_x-left_bound, t_y-lower_bound, z); 
        } 
    }
    Via_Direction cur_dir,next_dir ;
    int layer = min_cost_layer;
    // cout<<"min cost layer"<<layer<<endl;
    Bend* target_bend = new Bend(norm_x + left_bound, norm_y + lower_bound, layer);
    do{
        cur_dir = via_dir[norm_x][norm_y][layer];
        // cout<<"cur_dir: "<<cur_dir<<endl;

        switch(cur_dir){
            case(Pos_x):{
                next_dir = via_dir[ ++norm_x ][norm_y][layer];
                travel_idx++;
                break;
            }
            case(Neg_x):{
                next_dir = via_dir[ --norm_x ][norm_y][layer];
                travel_idx--;
                break;
            }
            case(Pos_y):{
                next_dir = via_dir[norm_x][ ++norm_y ][layer];
                travel_idx++;
                break;
            }
            case(Neg_y):{
                next_dir = via_dir[norm_x][ --norm_y ][layer];
                travel_idx--;
                break;
            }
            case(Pos_z):{
                layer += 2;
                next_dir = via_dir[norm_x][norm_y][layer];
                break;
            }
            case(Neg_z):{
                layer -= 2;
                next_dir = via_dir[norm_x][norm_y][layer];
                break;
            }    
        }
        if( cur_dir != next_dir ){
            // new bend construction
            // update demand map
            Bend* source_bend = new Bend(norm_x + left_bound, norm_y + lower_bound, layer, NULL, target_bend);
            target_bend->set_prev(source_bend);
            Segment new_segment(source_bend , target_bend);
            // new_segment.print();
            segment_of_netlist[ net_idx ].push_back( new_segment ); 
            target_bend = source_bend;
        }
        
    }    while((travel_idx != source_idx) );
    
    while ( layer != s_layer ){
        cur_dir = via_dir[norm_x][norm_y][layer];
        // cout<<"cur_dir: "<<cur_dir<<endl;
        switch(cur_dir){
            case(Pos_z):{
                layer += 1;
                next_dir = via_dir[norm_x][norm_y][layer];
                break;
            }
            case(Neg_z):{
                layer -= 1;
                next_dir = via_dir[norm_x][norm_y][layer];
                break;
            }    
        }
        if( cur_dir != next_dir ){
            // new bend construction
            // update demand map
            Bend* source_bend = new Bend(norm_x + left_bound, norm_y + lower_bound, layer);
            Segment new_segment(source_bend , target_bend);
            // new_segment.print();
            segment_of_netlist[ net_idx ].push_back( new_segment ); 
            target_bend = source_bend;
        }
    }
    Add_demand_3D(target_bend);
    if(t_layer_range.first == -1 ){
        t_layer_range.first = min_cost_layer;
        t_layer_range.second = min_cost_layer;
    }
    else{

    }
    // Bend* source_bend = new Bend(norm_x + left_bound, norm_y + lower_bound, layer);
    // Segment new_segment(source_bend , target_bend);
    // new_segment.print();
    // segment_of_netlist[ net_idx ].push_back( new_segment ); 

    cout<< "target layer: "<<min_cost_layer <<endl;
    
}

int Router::layer_assignment_straight_line_V(Bend* source, Bend* target, int min_layer, double range ){
    // int s_x = source->_x;
    // int s_y = source->_y;
    // int s_layer = source->_z;
    // int width = _placement->_boundary_width;
    // int height = _placement->_boundary_height;
    // int layer = _placement->_numLayers;
    // Congestion via_grid(width,height,layer,INFINITY);
    // via_grid(s_x,s_y,s_layer) =  0 ;
    // int t_x = target->_x;
    // int t_y = target->_y;
    // int t_z = target->_z;
    // bool limit_range = (t_z == -1 ? 0 : 1);

    // assert(s_x == t_x);
    // //from s_y goto t_y vertical line only go on odd layer
    // if(min_layer == -1){
    //     min_layer = 1; 
    // }
    // if(min_layer % 2 == 0){
    //     ++min_layer;
    // }
    // int x = s_x;
    // if(s_y > t_y){
    //     for(int y = s_y - 1 ; y > t_y; y--){
    //         vector<Via_Grid_cost> via_grid_set;
    //         for(int z = min_layer; z < layer; z+=2){
    //             if( (*supply_grid_map)(x,y,z) > (*demand_grid_map)(x,y,z) ){
    //                 via_grid(x,y,z) = via_grid(x,y+1,z); 
    //             }
    //             via_grid_set.push_back( Via_Grid_cost(&via_grid(x,y,z),z) );
    //             // min_via_cost = min(min_via_cost, via_grid(x,y,z));
    //         }
    //         // min_via_cost += 1;

    //         //update cost from neighbor
    //         sort( via_grid_set.begin(),via_grid_set.end(), Via_Grid_comparotor() ) ;
    //         int min_via_cost = via_grid_set.front(); 
    //         for(int i = 1 ; i < via_grid_set.size(); i++){
    //             int z_index = via_grid_set[i].z_index
    //             int upper_neighbor = z_index + 2;
    //             int lower_neighbor = z_index - 2;
    //             double upper_cost = (upper_neighbor < layer) ? via_grid(x,y,upper_neighbor) : INFINITY;
    //             double lower_cost = (lower_neighbor >= 0 ) ? via_grid(x,y,lower_neighbor) : INFINITY;
                
    //             via_grid(x,y,z_index) = min(via_grid(x,y,z_index), upper_cost, lower_cost);
    //         }
    //     }
    // }
}

int Router::layer_assignment_straight_line_H(Bend* source, Bend* target, int min_layer, double range ){
    
}

int Router::layer_assignment_straight_line_old_method(Bend* a, Bend*b , int m1){
    int layer = _placement->_numLayers;
    // cout<<"====================="<<endl;
    // cout<<"straight line"<<endl;
    // cout<<"from ("<<a->_x <<","<<a->_y<<") to ("<<b->_x <<","<<b->_y<<")"<<endl;
    if(a->_x == b->_x && a->_y == b->_y){
        if(m1 == -1){
            cout<<"hard to deal with"<<endl;
        }
        return m1;
        // for( int z = m1; z < layer ; z++){
        //     bool congestion = false;
        //     for( int y = y1 ; y < y2 ; y++){
        //         if( (*supply_grid_map)(x,y,z) - (*demand_grid_map)(x,y,z) < 1){
        //             congestion = true;
        //             break;
        //         }
        //     }
        //     if(!congestion){
        //         for( int y = y1 ; y < y2 ; y++){
        //             (*demand_grid_map)(x,y,z)++;
        //             return z;
        //         }
        //     }
        // }
    }
    if(a->_x == b->_x){   // vertical
        int y1, y2;
        int x = a->_x;
        y1 = a->_y;
        y2 = b->_y;
        if(y1 > y2) 
            swap(y1,y2);     // make y1 < y2
        if(m1 == -1 )   m1 = 1;
        if( m1 % 2 == 0 ) m1++;
        // cout<<"V "<<"m1 is "<<m1<<endl;
        for( int z = m1; z < layer ; z+=2){
            bool congestion = false;
            for( int y = y1 ; y < y2 ; y++){
                if( (*supply_grid_map)(x,y,z) - (*demand_grid_map)(x,y,z) < 1){
                    congestion = true;
                    break;
                }
            }
            if(!congestion){
                for( int y = y1 ; y < y2 ; y++){
                    (*demand_grid_map)(x,y,z)++;
                }
                return z;
            }
        }
    }
    if(a->_y == b->_y){   // Horizontal
        int x1, x2;
        int y = a->_y;
        x1 = a->_x;
        x2 = b->_x;
        if(x1 > x2)
            swap(x1,x2);   // make x1 < x2
        if(m1 == -1 )   m1 = 0;
        if( m1 % 2 == 1 ) m1++;
        // cout<<"H "<<"m1 is "<<m1<<endl;
        for( int z = m1; z < layer ; z+=2){
            bool congestion = false;
            for( int x = x1 ; x < x2 ; x++){
                if( (*supply_grid_map)(x,y,z) - (*demand_grid_map)(x,y,z) < 1){
                    congestion = true;
                    break;
                }
            }
            if(!congestion){
                for( int x = x1 ; x < x2 ; x++){
                    (*demand_grid_map)(x,y,z)++;
                }
                    return z;
            }
        }
        // (*supply_grid_map)(x,y,z);

    }
}

bool Router::check_demand(){
    int width =  _placement->_boundary_width;
    int height = _placement->_boundary_height;
    int layer = _placement->_numLayers;
    int s = 0;
    for( size_t x = 0 ; x < width; x++){
        for(size_t y = 0; y < height; y++){
            for(size_t z = 0; z < layer; z++){
                if( (*supply_grid_map)(x,y,z) < (*demand_grid_map)(x,y,z) ){
                    // cout<<"boom"<<endl;
                    s++;
                    // return false;
                }
            }
        }
    }
    cout<<"boom number"<<s<<endl;
    return true;
}

int Router::write_result(int net_idx , int& total_wire){
    int segment_size = segment_of_netlist[net_idx].size();
    int wire = 0;
    int segment = segment_size;
    int norm_x_factor = _placement->_leftBoundary;
    int norm_y_factor = _placement->_bottomBoundary;
    int norm_z_factor = 1;

    for(int i = 0 ; i < segment_size; i++){
        Bend* a = segment_of_netlist[net_idx][i].source;
        Bend* b = segment_of_netlist[net_idx][i].target;
        int a_x = a->_x + norm_x_factor;
        int a_y = a->_y + norm_y_factor;
        int a_z = a->_z + norm_z_factor;
        int b_x = b->_x + norm_x_factor;
        int b_y = b->_y + norm_y_factor; 
        int b_z = b->_z + norm_z_factor;
        wire += (abs(a_z - b_z) + abs(a_y - b_y) + abs(a_x - b_x) );
    }


    // cout<<_placement -> _netArray[net_idx] ->getName() <<endl; 
    // cout<<"total branch"<<branch_size<<endl;
    // if(branch_size == 1) return 0;
    // for( int i = 0; i < branch_size; i++){
    //     branch * trav = branch_of_netlist[net_idx][i];
    //     // cout<<trav->_x<<" "<<trav->_y<<" "<<trav->_z<<" ";
    //     int n = trav->_n;
    //     if( n >= branch_size) continue;
    //     assert( n < branch_size);
    //     branch * n_trav = branch_of_netlist[net_idx][n];
    //     // cout<<n_trav->_x<<" "<<n_trav->_y<<" "<<n_trav->_z<<" ";

    //     int a_x = trav->_x + norm_x_factor;
    //     int a_y = trav->_y + norm_y_factor;
    //     int a_z = trav->_z + norm_z_factor;
    //     int b_x = n_trav->_x + norm_x_factor;
    //     int b_y = n_trav->_y + norm_y_factor; 
    //     int b_z = n_trav->_z + norm_z_factor;
    //     // cout<<"the two point"<<endl;
    //     // cout<<a_y<<" "<<a_x<<" "<<a_z<<endl;
        
    //     // cout<<b_y<<" "<<b_x<<" "<<b_z<<endl;
    //     if(a_z == -1 || b_z == -1) continue;
    //     int z = max(a_z,b_z);
    //     if( z > _placement->_numLayers) {
    //         fail_segment++;
    //         continue;
    //     }
    //     if(a_x == b_x && a_y == b_y && a_z == b_z) continue;
    //     if(a_x == b_x && a_y == b_y ){
    //         wire += (labs(a_z - b_z));
    //         segment++;
    //         // cout<<wire<<endl;
    //     }
    //     if(a_y == b_y && a_z == b_z){
    //         wire += (labs(a_x - b_x));
    //         segment++;
    //         // cout<<wire<<endl<<endl;
    //     }
    //     if(a_x == b_x && a_z == b_z){
    //         wire += (labs(a_y - b_y));
    //         segment++;
    //         // cout<<wire<<endl<<endl;
    //     }
    // }
    total_wire += wire;
    // cout<<"total wirelength: "<<wire<<endl;
    return segment;
}

int Router::write_result_to_cmd(){
    int wire = 0;
    int segment = 0;
    fail_segment = 0;
    for(int i = 0 ; i < two_pin_netlist.size(); i++){
        segment += write_result(i,wire);
    }
    cout<<"total wirelength: "<<wire<<endl;
    cout<<"segment is: "<<segment<<endl;
    cout<<"fail semgent is "<<fail_segment<<endl;
    cout<<"completed rate:"<<double(segment)/double(segment+fail_segment)<<endl;
    return segment;
}

void Router::writeResult(fstream &outFile){
    outFile<<"NumRoutes "<<write_result_to_cmd()<<endl;
    for(int i = 0 ; i < two_pin_netlist.size(); i++){
        writeResult(i,outFile);
    }
}

void Router::writeResult(int net_idx,fstream &outFile ){
    int segment_size = segment_of_netlist[net_idx].size();
    int wire = 0;
    int segment = 0;
    int norm_x_factor = _placement->_leftBoundary;
    int norm_y_factor = _placement->_bottomBoundary;
    int norm_z_factor = 1;

    for(int i = 0 ; i < segment_size; i++){
        Bend* a = segment_of_netlist[net_idx][i].source;
        Bend* b = segment_of_netlist[net_idx][i].target;
        int a_x = a->_x + norm_x_factor;
        int a_y = a->_y + norm_y_factor;
        int a_z = a->_z + norm_z_factor;
        int b_x = b->_x + norm_x_factor;
        int b_y = b->_y + norm_y_factor; 
        int b_z = b->_z + norm_z_factor;
        outFile<<a_y<<" "<<a_x<<" "<<a_z<<" ";
        outFile<<b_y<<" "<<b_x<<" "<<b_z<<" ";
        outFile<<_placement -> _netArray[net_idx] ->getName() <<endl; 
    }
    
    // if(branch_size == 1) return ;
    // for( int i = 0; i < branch_size; i++){
    //     branch * trav = branch_of_netlist[net_idx][i];
    //     // cout<<trav->_x<<" "<<trav->_y<<" "<<trav->_z<<" ";
    //     int n = trav->_n;
    //     if( n >= branch_size) continue;
    //     assert( n < branch_size);
    //     branch * n_trav = branch_of_netlist[net_idx][n];
    //     // cout<<n_trav->_x<<" "<<n_trav->_y<<" "<<n_trav->_z<<" ";

    //     int a_x = trav->_x + norm_x_factor;
    //     int a_y = trav->_y + norm_y_factor;
    //     int a_z = trav->_z + norm_z_factor;
    //     int b_x = n_trav->_x + norm_x_factor;
    //     int b_y = n_trav->_y + norm_y_factor; 
    //     int b_z = n_trav->_z + norm_z_factor;
    //     // cout<<"the two point"<<endl;
    //     if(a_z == -1 || b_z == -1) continue;
    //     int z = max(a_z,b_z);
    //     if( z > _placement->_numLayers) continue;
    //     if(a_x == b_x && a_y == b_y && a_z == b_z) continue;
        
    //     outFile<<a_y<<" "<<a_x<<" "<<a_z<<" ";
    //     outFile<<b_y<<" "<<b_x<<" "<<b_z<<" ";
    //     outFile<<_placement -> _netArray[net_idx] ->getName() <<endl; 
    // }
}
