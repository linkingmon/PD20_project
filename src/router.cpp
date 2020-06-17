#include "router.h"
#include "net.h"
#include "struc.h"
#include "congestion.h"

using namespace std;

void Router::construct_supply_demand_map(){
    // vector<NonDefault*> nondefault;
    // grid_map = new congestion()
    
    // supply map construct
    // defult supply
    int layer_num = _placement->_numLayers;
    int y = _placement->_boundary_height;
    int norm_y_factor = _placement->_leftBoundary;
    int x = _placement->_boundary_width;
    int norm_x_factor = _placement->_bottomBoundary;
    int norm_z_factor = 1 ;
    // y -= norm_y_factor;
    // x -= norm_x_factor;
    for(size_t layer = 0 ; layer < layer_num ; layer++){
        int default_supply = _placement->layers[layer]->get_supply();
        vector<vector<double>> one_layer_cong = (vector<vector<double>>(y , vector<double>(x,default_supply)) );
        supply_grid_map->add_congestion_map(one_layer_cong);
    }
    // non_defult supply 
    size_t non_default_num = _placement->_numNonDefault;
    for(size_t i = 0 ; i < non_default_num ; i++){
        int x = _placement->nondefault[i]->_x - norm_x_factor;
        int y = _placement->nondefault[i]->_y - norm_y_factor;
        int z = _placement->nondefault[i]->_z - norm_z_factor;
        int offset = _placement->nondefault[i]->_offset;
        cout<<"x is "<<x<<" y is "<<y<<" z is "<<z<<" offset is " <<offset<<endl;
        (*supply_grid_map)(x,y,z) += offset;
    }

    // add block demand
    size_t cell_num = _placement->_numCells;
    for(size_t i = 0 ; i < cell_num ; i++){
        size_t block_layer ;
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
    coordinate* sink = new coordinate(a_x,a_y,z); 
    L_net->set_source ( new coordinate(a_x,a_y,z) );
    coordinate* L_bend = L_route_2D(a_x,b_x,a_y,b_y,z);
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
    coordinate* sink = new coordinate(a_x,a_y,z); 
    coordinate* Z_bend;
    Z_bend = Z_routing(a_x,b_x,a_y,b_y,z);

    Z_net->set_source ( sink );
    sink->set_next(Z_bend);
    Z_bend->set_prev(sink);

    twopin_netlist_Z.push_back(Z_net);
}

coordinate* Router::Z_routing(size_t x1, size_t x2, size_t y1 ,size_t y2 ,size_t z){

    double z_cost_h ,z_cost_v ;
    coordinate* Z_bend_h;
    coordinate* Z_bend_v;
    coordinate* sink = new coordinate(x1,y1,z); 
    // Z_net->set_source ( new coordinate(x1,y1,z) );
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
        coordinate* temp = Z_bend_h->get_next();
        delete temp;
        delete Z_bend_h;
        
        return Z_bend_v;
    }
    else{
        sink->set_next(Z_bend_h);
        Z_bend_h->set_prev(sink);
        coordinate* temp = Z_bend_v->get_next();
        delete temp;
        delete Z_bend_v;

        return Z_bend_h;
    } 
    
}

double Router::Z_route_2D_V(size_t x1, size_t x2, size_t y1, size_t y2 , size_t z , coordinate*& Z_bend){
    double best_cost = 1e10;
    coordinate* Z_bend_n;
    
    //asssume x1 < x2
    bool inv = false;
    if( x1 > x2 ){
        swap(x1,x2);
        swap(y1,y2);
        inv = true;
    }

    double best_x = 0 ;

    for (size_t x = x1+1 ; x <= x2; x++){
        double cost = V_route(x1,x,y1,z) + H_route(x,y1,y2,z) + V_route(x,x2,y2,z);
        if( cost < best_cost ){
            // delete Z_bend;
            // Z_bend = new coordinate(x,y1,z);
            best_cost = cost;
            best_x = x;
        }
    }
    if( inv == false){
        Z_bend = new coordinate(best_x,y1,z);
        Z_bend_n = new coordinate(best_x,y2,z);
        Z_bend->set_next( Z_bend_n );
        Z_bend_n->set_prev(Z_bend); 
    }
    else{
        if( best_x != x2){
            Z_bend = new coordinate(best_x,y2,z);
            Z_bend_n = new coordinate(best_x,y1,z);
            Z_bend->set_next( Z_bend_n );
            Z_bend_n->set_prev(Z_bend);
        }
        else    // L routing only one bend exist
            Z_bend = new coordinate(best_x,y1,z);
    }
    return best_cost;
}

double Router::Z_route_2D_H(size_t x1, size_t x2, size_t y1, size_t y2 , size_t z , coordinate*& Z_bend){
    double best_cost = 1e10;
    coordinate* Z_bend_n;
    //asssume y1 < y2
    bool inv = false;
    if(y1 > y2){
        swap(y1,y2);
        swap(x1,x2);
        inv = true;
    }

    double best_y = 0;
    for (size_t y = y1+1 ; y <= y2; y++){
        double cost = H_route(x1,y1,y,z) + V_route(x1,x2,y,z) + H_route(x2,y,y2,z);
        if( cost < best_cost ){
            // delete Z_bend;
            // Z_bend = new coordinate(x1,y,z);
            best_cost = cost;
            best_y = y;
        }
    }

    if( inv == false){
        Z_bend = new coordinate(x1,best_y,z);
        Z_bend_n = new coordinate(x2,best_y,z);
        Z_bend->set_next( Z_bend_n );
        Z_bend_n->set_prev(Z_bend); 
    }
    else{
        if(best_y != y2 ){
            Z_bend = new coordinate(x2,best_y,z);
            Z_bend_n = new coordinate(x1,best_y,z);
            Z_bend->set_next( Z_bend_n );
            Z_bend_n->set_prev(Z_bend);
        }
        else {  //L routing only one bend exist
            Z_bend = new coordinate(x1,best_y,z);
        }
    }
    return best_cost;
     
}
coordinate* Router::L_route_2D(size_t x1, size_t x2, size_t y1, size_t y2 , size_t z){
    double L1_cost = 0;
    double L2_cost = 0;
    // int z = 0 ; 
    L1_cost = H_route(x1,y1,y2,z) + V_route(x1,x2,y2,z);
    L2_cost = V_route(x1,x2,y1,z) + H_route(x2,y1,y2,z);

    coordinate* L_bend = new coordinate(x1,y2,z) ; // L1_cost > L2_cost
    if(L1_cost < L2_cost ){
        delete L_bend;
        L_bend = new coordinate(x2,y1,z);
    }
    else if(L1_cost == L2_cost){
        if( rand()%2 == 0){
            delete L_bend;
            L_bend = new coordinate(x2,y1,z);
        } 
    }
    coordinate* target = new coordinate(x2,y2,z);
    L_bend->set_next( target );
    target->set_prev( L_bend );

    return L_bend;
 
}

//y1 to y2 
double Router::H_route(size_t x , size_t y1, size_t y2 , size_t z ){
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

double Router::V_route(size_t x1 , size_t x2, size_t y , size_t z ){
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