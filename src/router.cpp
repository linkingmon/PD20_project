#include <stdio.h>
#include <stdlib.h>
#include "router.h"
#include "net.h"
#include "struc.h"
#include "congestion.h"

using namespace std;

// to do list
// 2D projection check
// expansion check
// stenier structrue check 
// congestion map update when path is determined (add/exclude) check
// layer assignment

void Router::route(){
    
    cout << "Routing ..." << endl;
    // cout<<(*row_map)(2,3,1)<<endl;

    cost_row_map->print_congestion();
    cost_col_map->print_congestion();
    
    construct_grid_map();
    construct_supply_demand_map();
    supply_grid_map->print_congestion();
    demand_grid_map->print_congestion();

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
    
    construct_congestion_map();
    A_star_search_routing();
    layer_assignment(); 
    write_result_to_cmd();
    
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
    cout<<"default supply"<<endl;
    for(size_t layer = 0 ; layer < layer_num ; layer++){
        int default_supply = _placement->layers[layer]->get_supply();
        vector<vector<double>> one_layer_cong = (vector<vector<double>>(width , vector<double>(height,default_supply)) );
        supply_grid_map->set_congestion_map(layer,one_layer_cong);
    }

    // non_defult supply 
    cout<<"non default supply"<<endl;
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

    branch<int>* branch_a = new branch<int>(a_x,a_y,a_z);
    branch<int>* branch_b = new branch<int>(b_x,b_y,b_z);
    two_pin_net<int>* L_net = new two_pin_net<int>(branch_a,branch_b);
    
    int z = 0;
    Bend* sink = new Bend(a_x,a_y,z); 
    L_net->set_source ( new Bend(a_x,a_y,z) );
    Bend* L_bend = L_route_2D(a_x,b_x,a_y,b_y,z);
    sink->set_next(L_bend);
    L_bend->set_prev(sink);

    twopin_netlist_L.push_back(L_net);
    
}

void Router::two_pin_net_Both_L_routing( two_pin_net<int>* a , bool first_round){

    branch<int>* branch_a = a->b_source;
    branch<int>* branch_b = a->b_target;
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

void Router::two_pin_net_L_routing( two_pin_net<int>* a ){

    branch<int>* branch_a = a->b_source;
    branch<int>* branch_b = a->b_target;
    // two_pin_net<int>* L_net = new two_pin_net<int>(branch_a,branch_b);
    int a_x = branch_a->_x;
    int a_y = branch_a->_y;
    int a_z = branch_a->_z;
    int b_x = branch_b->_x;
    int b_y = branch_b->_y;
    int b_z = branch_b->_z;
    int z = 0;


    Bend* sink = new Bend(a_x,a_y,z); 
;
    
    Bend* L_bend = L_route_2D(a_x,b_x,a_y,b_y,z);
    if( L_bend->_x == a_x && L_bend->_y == a_y){
        sink = L_bend;    
    }
    else{
        sink->set_next(L_bend);
        L_bend->set_prev(sink);
    }
    a->set_source(sink);
    a->print_bend();
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
    branch<int>* branch_a = new branch<int>(a_x,a_y,a_z);
    branch<int>* branch_b = new branch<int>(b_x,b_y,b_z);
    two_pin_net<int>* Z_net = new two_pin_net<int>(branch_a,branch_b);
    
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

//y1 to y2 
double Router::V_route(size_t x , size_t y1, size_t y2 , size_t z ){

    return V_route_edge( x ,  y1,  y2 ,  z );
    return V_route_grid( x ,  y1,  y2 ,  z );
}

//x1 to x2
double Router::H_route(size_t x1 , size_t x2, size_t y , size_t z ){
    
    return H_route_edge( x1 ,  x2,  y ,  z );
    return H_route_grid( x1 ,  x2,  y ,  z );
}


// y1 to y2 computed by edge 
double Router::V_route_edge(size_t x , size_t y1, size_t y2 , size_t z ){
    double h_cost = 0;
    // assume y1 < y2 
    if(y1 > y2)
        swap(y1,y2);
    for(size_t y = y1; y < y2 ; y++){
        h_cost += (*cost_col_map)(x,y,z);
        cout<<x<<" "<<y<<" "<<z<<endl;
        // cout<<h_cost<<endl;
    }
    cout<<"horizontal cost"<<h_cost<<endl;
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

//y1 to y2 compute_by grid
double Router::V_route_grid(size_t x , size_t y1, size_t y2 , size_t z ){
    double h_cost = 0;
    // assume y1 < y2 
    if(y1 > y2)
        swap(y1,y2);
    for(size_t y = y1; y < y2 ; y++){
        h_cost += (*cost_row_map)(x,y,z);
        cout<<x<<" "<<y<<" "<<z<<endl;
        // cout<<h_cost<<endl;
    }
    cout<<"horizontal cost"<<h_cost<<endl;
    return h_cost;
}

//x1 to x2 compute_by edge
double Router::H_route_edge(size_t x1 , size_t x2, size_t y , size_t z ){
    double v_cost = 0;
    // assume x1 < x2
    if(x1 > x2)
        swap(x1,x2); 
        cout<<"x2 is "<<x2<<endl;
        cout<<"x1 is "<<x1<<endl;
    for(size_t x = x1; x < x2 ; x++){
        cout<<x<<" "<<y<<" "<<z<<endl;
        v_cost += (*cost_row_map)(x,y,z);
    }
    cout<<"vertival cost"<<v_cost<<endl;
    return v_cost;
}

//x1 to x2 compute_by grid
double Router::H_route_grid(size_t x1 , size_t x2, size_t y , size_t z ){
    double v_cost = 0;
    // assume x1 < x2
    if(x1 > x2)
        swap(x1,x2); 
    for(size_t x = x1; x < x2 ; x++){
        cout<<x<<" "<<y<<" "<<z<<endl;
        v_cost += (*cost_col_map)(x,y,z);
    }
    cout<<"vertival cost"<<v_cost<<endl;
    return v_cost;
}

void Router::maze_routing(){

}

void Router::A_star_search_routing(){
    
    for(size_t i = 0 ; i < two_pin_netlist.size(); i++){
        for(size_t j = 0; j < two_pin_netlist[i].size();j++){
            int s_x = two_pin_netlist[i][j].b_source->_x;
            int s_y = two_pin_netlist[i][j].b_source->_y;
            int t_x = two_pin_netlist[i][j].b_target->_x;
            int t_y = two_pin_netlist[i][j].b_target->_y;
            
            if( abs(s_x-t_x) <= 0) continue;
            if( abs(s_y-t_y) <= 0) continue;
            Shortest_Path test( s_x, s_y, 0, t_x, t_y, 0, 1, cost_row_map, cost_col_map, demand_grid_map );
            test.Dijkstra();
            test.Build_the_path();
            Bend* result = test.target();
            two_pin_netlist[i][j].set_source(result);
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
    printf("FLUTE wirelength = %lf\n", flutetree.length);
    printtree(flutetree);

    flutewl = flute_wl(d, x, y, ACCURACY);
    printf("FLUTE wirelength (without RSMT construction) = %lf\n", flutewl);

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
    Tree t = Flute_function(x,y);    
    int i;
    double p1_x,p1_y,p2_x,p2_y;
    two_pin_net<int> tp_net;
    for (i=0; i<2*t.deg-2; i++){
        p1_x = t.branch[i].x;
        p1_y = t.branch[i].y;
        int n = t.branch[i].n;
        double z = -1;
        branch<int>* b = new branch<int>( p1_x, p1_y, z, i, n); 
        branch_of_netlist[idx].push_back(b);
    }
    for (i=0; i<2*t.deg-2; i++){
        p1_x = t.branch[i].x;
        p1_y = t.branch[i].y;
        int n = t.branch[i].n;
        if( n == i) continue;
        p2_x = t.branch[n].x;
        p2_y = t.branch[n].y;
        // if( p1_x == p2_x && p2_x == p2_y) continue;
        double z = 0;
        branch<int>* b1 = branch_of_netlist[idx][i];
        branch<int>* b2 = branch_of_netlist[idx][n];
        two_pin_net<int> tp_net(b1,b2);
        two_pin_netlist[idx].push_back(tp_net);
    }

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

void Router::Add_demand_H( int x1, int x2, int y, int z, double f){
    if(x1 > x2) 
        swap(x1,x2); //make x1 < x2
    for(int x = x1; x < x2; x++){
        (*demand_row_map)(x,y,z) += f;
    }
}

void Router::Add_demand_V( int x, int y1, int y2, int z, double f){
    if(y1 > y2) 
        swap(y1,y2); //make y1 < y2
    for(int y = y1; y < y2; y++){
        (*demand_col_map)(x,y,z) += f;
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


//2D
void Router::supply_from_grid_to_edge(){
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
    for(int i = 0 ; i < two_pin_netlist.size(); i++){
        layer_assignment_one_net(i);
        
    }
}

void Router::layer_assignment_one_net(int idx){
    // two_pin_netlist[idx];//vector<two_pin_net><int>
    int branch_idx = branch_of_netlist[idx].size();
    cout<<_placement -> _netArray[idx] ->getName()<<endl;
    for(int i = 0 ; i < two_pin_netlist[idx].size(); i++){
        two_pin_netlist[idx][i].print_bend();
        if( i < _placement -> _netArray[idx] -> getPin_num())
            two_pin_netlist[idx][i].b_source ->_z = _placement -> _netArray[idx] -> getPin(i) -> get_layer();
        if( *(two_pin_netlist[idx][i].b_source) == *(two_pin_netlist[idx][i].b_target))
            continue;
        Bend* temp = two_pin_netlist[idx][i].get_source();
        Bend* n_temp = temp -> get_next();
    
        int min_layer = _placement -> _netArray[idx] -> getMinLayer();
        int routing_layer = layer_assignment_straight_line(temp, n_temp, min_layer);
        cout<<"routing layer: "<<routing_layer<<endl;
        if( i < _placement -> _netArray[idx] -> getPin_num()){
            // int target_idx = two_pin_netlist[idx][i] -> b_target ->_n;
            int source_idx = two_pin_netlist[idx][i].b_source ->_id;
            int source_layer = _placement -> _netArray[idx] -> getPin(source_idx) -> get_layer();
            int first_idx;
            cout<<"source_idx: "<<source_idx<<endl;
            cout<<"source layer: "<<source_layer<<endl;
            two_pin_netlist[idx][i].b_source ->_z = source_layer;
            branch<int> * trav = two_pin_netlist[idx][i].b_source;
            cout<<"source in two pin net: "<<trav->_x<<" "<<trav->_y<<" "<<trav->_z<<" "<<endl;
            trav = branch_of_netlist[idx][i];
            cout<<"source in branch list: "<<trav->_x<<" "<<trav->_y<<" "<<trav->_z<<" "<<endl;
            if( source_layer != routing_layer ){
                int x = two_pin_netlist[idx][i].b_source -> _x;
                int y = two_pin_netlist[idx][i].b_source -> _y;
                branch<int>* n_branch = new branch<int>( x,y,routing_layer,branch_idx,source_idx);
                two_pin_netlist[idx][i].b_source->_n = branch_idx;
                branch_of_netlist[idx].push_back(n_branch);
                branch<int> * trav = n_branch;
                cout<<"branch_idx: "<<branch_idx<<" "<<trav->_x<<" "<<trav->_y<<" "<<trav->_z<<" "<<endl;
                branch_idx++;
                
                x = n_temp -> _x;
                y = n_temp -> _y;
                
                branch<int>* n2_branch = new branch<int>( x,y,routing_layer,branch_idx,branch_idx-1);
                branch_of_netlist[idx].push_back(n2_branch);
                trav = n2_branch;
                cout<<"branch_idx: "<<branch_idx<<" "<<trav->_x<<" "<<trav->_y<<" "<<trav->_z<<" "<<endl;
                branch_idx++;
            }
            else{                
                int x = n_temp -> _x;
                int y = n_temp -> _y;
                branch<int>* n_branch = new branch<int>( x,y,routing_layer,branch_idx,source_idx);
                branch_of_netlist[idx].push_back(n_branch);
                branch<int> * trav = n_branch;
                cout<<"branch_idx: "<<branch_idx<<" "<<trav->_x<<" "<<trav->_y<<" "<<trav->_z<<" "<<endl;
                two_pin_netlist[idx][i].b_source->_n = branch_idx;
                branch_idx++;
               
            }
        }

        temp = n_temp;
        n_temp = temp->get_next(); 

        while(n_temp != NULL){  //until n_temp is target
            routing_layer = layer_assignment_straight_line(temp,n_temp,min_layer);
            cout<<"routing layer: "<<routing_layer<<endl;
            int x = temp -> _x;
            int y = temp -> _y;
            branch<int>* n_branch = new branch<int>( x,y,routing_layer,branch_idx,branch_idx-1) ;
            branch_of_netlist[idx].push_back(n_branch);
            branch_idx++;
            
            branch<int> * trav = n_branch;
            cout<<trav->_x<<" "<<trav->_y<<" "<<trav->_z<<" "<<endl;
            x = n_temp -> _x;
            y = n_temp -> _y;
            branch<int>* n2_branch = new branch<int>( x,y,routing_layer,branch_idx,branch_idx-1) ;
            branch_of_netlist[idx].push_back(n2_branch);
            branch_idx++;
            temp = n_temp;
            n_temp = temp->get_next(); 
            trav = n2_branch;
            cout<<trav->_x<<" "<<trav->_y<<" "<<trav->_z<<" "<<endl;
            
        }
        int target_idx = two_pin_netlist[idx][i].b_target ->_id;
        cout<<"target_idx "<<target_idx<<endl;
        int target_layer = two_pin_netlist[idx][i].b_target ->_z;
        // int target_layer;
        if( target_idx < _placement -> _netArray[idx] -> getPin_num()){
            target_layer = _placement -> _netArray[idx] -> getPin(target_idx) -> get_layer();
        }
        if( target_layer == -1){
            // int x = two_pin_netlist[idx][i].b_target -> _x;
            // int y = two_pin_netlist[idx][i].b_target -> _y;
            // branch<int>* n_branch = new branch<int>( x,y,routing_layer,branch_idx,target_idx);
            // branch_of_netlist[idx].push_back(n_branch);
            // branch_idx++;
            two_pin_netlist[idx][i].b_target ->_z = routing_layer;
            // branch<int> * trav = n_branch;
            // cout<<trav->_x<<" "<<trav->_y<<" "<<trav->_z<<" "<<endl;
        }
        else if( target_layer != routing_layer ){
            int x = two_pin_netlist[idx][i].b_target -> _x;
            int y = two_pin_netlist[idx][i].b_target -> _y;
            branch<int>* n_branch = new branch<int>( x,y,routing_layer,branch_idx,target_idx);
            branch_of_netlist[idx].push_back(n_branch);
            branch_idx++;
        }
    }
}

int Router::layer_assignment_straight_line(Bend* a, Bend*b , int m1){
    int layer = _placement->_numLayers;
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
        if(m1 == -1 )   m1 = 0;
        if( m1 % 2 == 1 ) m1++;
        cout<<"V "<<"m1 is "<<m1<<endl;
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
                    return z;
                }
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
        if(m1 == -1 )   m1 = 1;
        if( m1 % 2 == 0 ) m1++;
        cout<<"H "<<"m1 is "<<m1<<endl;
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
                    return z;
                }
            }
        }
        // (*supply_grid_map)(x,y,z);

    }
}

void Router::write_result(int net_idx){
    int branch_size = branch_of_netlist[net_idx].size();
    for( int i = 0; i < branch_size; i++){
        branch<int> * trav = branch_of_netlist[net_idx][i];
        cout<<trav->_x<<" "<<trav->_y<<" "<<trav->_z<<" ";
        int n = trav->_n;
        branch<int> * n_trav = branch_of_netlist[net_idx][n];
        cout<<n_trav->_x<<" "<<n_trav->_y<<" "<<n_trav->_z<<" ";
        cout<<_placement -> _netArray[net_idx] ->getName() <<endl; 
    }
}

void Router::write_result_to_cmd(){
    for(int i = 0 ; i < two_pin_netlist.size(); i++){
        write_result(i);
        
    }
}