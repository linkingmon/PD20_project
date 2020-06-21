#include "router.h"
#include "net.h"
#include "struc.h"
#include "congestion.h"

using namespace std;

void Router::route(){
    
    cout << "Routing ..." << endl;
    cout<<(*row_map)(2,3,1)<<endl;

    row_map->print_congestion();
    col_map->print_congestion();
    
    construct_grid_map();
    construct_supply_demand_map();
    supply_grid_map->print_congestion();
    demand_grid_map->print_congestion();
    
    Shortest_Path test( 0, 0, 0, 4, 4, 0, 0, row_map, col_map, demand_grid_map );
    // Shortest_Path(int s_x , int s_y , int s_z , int t_x , int t_y , int t_z , double a_factor, 
    //     Congestion_Row* r_map , Congestion_Col* c_map , Congestion* g_map )
    //     :source_x(s_x), source_y(s_y) , source_z(s_z) , target_x(t_x) , target_y(t_y) , target_z(t_z),
    // expand_factor(a_factor), row_map(r_map), col_map(c_map), grid_map(g_map)
    test.Dijkstra();
    
}

void Router::construct_grid_map(){
    int width =  _placement->_boundary_width;
    int height = _placement->_boundary_height;
    int MCell_num = _placement->_numMasterCell;
    int cell_num = _placement->_numCells;
    int norm_x_factor = _placement->_bottomBoundary;
    int norm_y_factor = _placement->_leftBoundary;

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

    two_pin_net* L_net = new two_pin_net(a,b);
    
    int z = 0;
    bend* sink = new bend(a_x,a_y,z); 
    L_net->set_source ( new bend(a_x,a_y,z) );
    bend* L_bend = L_route_2D(a_x,b_x,a_y,b_y,z);
    sink->set_next(L_bend);
    L_bend->set_prev(sink);

    twopin_netlist_L.push_back(L_net);
    
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
    two_pin_net* Z_net = new two_pin_net(a,b);
    bend* sink = new bend(a_x,a_y,z); 
    bend* Z_bend;
    Z_bend = Z_routing(a_x,b_x,a_y,b_y,z);

    Z_net->set_source ( sink );
    sink->set_next(Z_bend);
    Z_bend->set_prev(sink);

    twopin_netlist_Z.push_back(Z_net);
}

bend* Router::Z_routing(size_t x1, size_t x2, size_t y1 ,size_t y2 ,size_t z){

    double z_cost_h ,z_cost_v ;
    bend* Z_bend_h;
    bend* Z_bend_v;
    bend* sink = new bend(x1,y1,z); 
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
        bend* temp = Z_bend_h->get_next();
        delete temp;
        delete Z_bend_h;
        
        return Z_bend_v;
    }
    else{
        sink->set_next(Z_bend_h);
        Z_bend_h->set_prev(sink);
        bend* temp = Z_bend_v->get_next();
        delete temp;
        delete Z_bend_v;

        return Z_bend_h;
    } 
    
}

double Router::Z_route_2D_H(size_t x1, size_t x2, size_t y1, size_t y2 , size_t z , bend*& Z_bend){
    double best_cost = 1e10;
    bend* Z_bend_n;
    
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
        Z_bend = new bend(best_x,y1,z);
        Z_bend_n = new bend(best_x,y2,z);
        Z_bend->set_next( Z_bend_n );
        Z_bend_n->set_prev(Z_bend); 
    }
    else{
        if( best_x != x2){
            Z_bend = new bend(best_x,y2,z);
            Z_bend_n = new bend(best_x,y1,z);
            Z_bend->set_next( Z_bend_n );
            Z_bend_n->set_prev(Z_bend);
        }
        else    // L routing only one bend exist
            Z_bend = new bend(best_x,y1,z);
    }
    return best_cost;
}

double Router::Z_route_2D_V(size_t x1, size_t x2, size_t y1, size_t y2 , size_t z , bend*& Z_bend){
    double best_cost = 1e10;
    bend* Z_bend_n;
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
        Z_bend = new bend(x1,best_y,z);
        Z_bend_n = new bend(x2,best_y,z);
        Z_bend->set_next( Z_bend_n );
        Z_bend_n->set_prev(Z_bend); 
    }
    else{
        if(best_y != y2 ){
            Z_bend = new bend(x2,best_y,z);
            Z_bend_n = new bend(x1,best_y,z);
            Z_bend->set_next( Z_bend_n );
            Z_bend_n->set_prev(Z_bend);
        }
        else {  //L routing only one bend exist
            Z_bend = new bend(x1,best_y,z);
        }
    }
    return best_cost;
     
}
bend* Router::L_route_2D(size_t x1, size_t x2, size_t y1, size_t y2 , size_t z){
    double L1_cost = 0;
    double L2_cost = 0;
    // int z = 0 ; 
    L1_cost = V_route(x1,y1,y2,z) + H_route(x1,x2,y2,z);
    L2_cost = H_route(x1,x2,y1,z) + V_route(x2,y1,y2,z);

    bend* L_bend = new bend(x1,y2,z) ; // L1_cost > L2_cost
    if(L1_cost < L2_cost ){
        delete L_bend;
        L_bend = new bend(x2,y1,z);
    }
    else if(L1_cost == L2_cost){
        if( rand()%2 == 0){
            delete L_bend;
            L_bend = new bend(x2,y1,z);
        } 
    }
    bend* target = new bend(x2,y2,z);
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
        h_cost += (*row_map)(x,y,z);
        cout<<x<<" "<<y<<" "<<z<<endl;
        // cout<<h_cost<<endl;
    }
    cout<<"horizontal cost"<<h_cost<<endl;
    return h_cost;
}


//y1 to y2 compute_by grid
double Router::V_route_grid(size_t x , size_t y1, size_t y2 , size_t z ){
    double h_cost = 0;
    // assume y1 < y2 
    if(y1 > y2)
        swap(y1,y2);
    for(size_t y = y1; y < y2 ; y++){
        h_cost += (*row_map)(x,y,z);
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
    for(size_t x = x1; x < x2 ; x++){
        cout<<x<<" "<<y<<" "<<z<<endl;
        v_cost += (*col_map)(x,y,z);
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
        v_cost += (*col_map)(x,y,z);
    }
    cout<<"vertival cost"<<v_cost<<endl;
    return v_cost;
}

void Router::maze_routing(){

}

void Router::A_star_search_routing(){

}