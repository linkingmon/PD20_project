#ifndef CONGESTION_H
#define CONGESTION_H

#include <iostream>
#include <vector>
#include <iomanip> 
using namespace std;

// describe congestion on grid
class Congestion{

public:
    Congestion() {}
    Congestion(size_t x, size_t y, size_t z , double default_supply = 0)
    :Col(x), Row(y), Layer(z){ 
        // cong_map.resize( x * y * z );
        // cong_map_3D.clear();
        cong_map_3D = vector<vector<vector<double>>> ( z , vector<vector<double>>(y , vector<double>(x,default_supply)) );
        
    }

    double& operator() (size_t x , size_t y, size_t z ){
        // return cong_map.at(x + y * Row + z * Row * Col );
        return cong_map_3D[z][y][x];
    }
    // set congestion map with one layer 
    void set_congestion_map( size_t layer, vector<vector<double>> one_layer_cong  ) {
        // cout<<"bug is me"<<endl;
        // cout<<one_layer_cong.size()<<endl;
        // cout<<cong_map_3D[layer].size()<<endl;
        cong_map_3D[layer] = one_layer_cong ;
        // cout<<"bug is me"<<e1ndl;
    }

    void cong_map_clear(){  cong_map_3D.clear();}

    void print_congestion(){
        cout<<endl;
        cout<<"====================================="<<endl;
        cout<<"congestion grid"<<endl;
        for(size_t i = 0 ; i < Layer ; i++){
            cout<<"layer : "<<i<<endl;
            // int z = 0;
            cout<<setw(5)<<"";
            for(size_t k = 0 ; k < Col ; k++){
                cout<<setw(4)<<"Col"<<k+1;
            }
            cout<<endl;
            for(size_t j = 0 ; j < Row ; j++){
                cout<<"Row"<<j+1<<" ";
                for(size_t k = 0; k < Col ; k++){
                    // cout<<setw(5)<<cong_map.at( i * Row * Col + j * Col + k);
                    cout<<setw(5)<<cong_map_3D[i][j][k];
                }
                cout<<endl;
            }
            cout<<endl;
        }
        cout<<"====================================="<<endl;
        cout<<endl;
    }   
private: 
    vector<double> cong_map;
    vector<vector<vector<double>>> cong_map_3D;
    size_t Row;
    size_t Col;
    size_t Layer;
};

//describe congestion of Row edge ( Horizontal edge )
class Congestion_Row {
public:
    Congestion_Row() {}
    Congestion_Row(size_t x , size_t y , size_t z)
    :Col(x-1),Row(y), Layer(z){
        // cong_map.resize( x * (y-1) * z );
        // vectorM
        cong_map_3D = vector<vector<vector<double>>> ( x , vector<vector<double>>(y , vector<double>(z,1)) );
    }

    double& operator() (size_t x , size_t y, size_t z ){
        // return cong_map.at(x*Col + y + z * Row * Col );
        return cong_map_3D[x][y][z];
    }

    double& block_to_edge_p(size_t x, size_t y , size_t z){         //positive direction in Row
        if( x >= Col ){
            cerr<<"wrong positive direction in Horizontal"<<endl;
            exit(0);
        }
        // return cong_map.at( z * Row * Col + y * Row + x);
        return cong_map_3D[x][y][z];
    }
    
    double& block_to_edge_n(size_t x, size_t y , size_t z){         //negative direction in Row
        if( x > Col || x <= 0 ){
            cerr<<"wrong negative direction in Horizontal"<<endl;
            exit(0);
        }
        // return cong_map.at( z * Row * Col + (y-1) * Row + x);
        return cong_map_3D[x][y-1][z];
    }

    void print_congestion(){
        cout<<endl;
        cout<<"====================================="<<endl;
        cout<<"congestion Row"<<endl;
        for(size_t i = 0 ; i < Layer ; i++){
            cout<<"layer : "<<i<<endl;
            int z = 0;
            for(size_t j = 0 ; j < Row ; j++){
                cout<<"Row"<<j<<" ";
                for(size_t k = 0; k < Col ; k++){
                    // cout<<setw(5)<<cong_map.at( i * Row * Col + j * Col + k);
                    // cong_map_3D[j][k][i] = 1;
                    z++;
                    cout<<setw(5)<<cong_map_3D[k][j][i];
                }
                cout<<endl;
            }
            cout<<endl;
        }
        cout<<"====================================="<<endl;
        cout<<endl;
    }   
private:
    vector<double> cong_map;
    vector<vector<vector<double>>> cong_map_3D;
    size_t Row;
    size_t Col;
    size_t Layer;
};


//describe congestion on Column edge ( vertical edge )
class Congestion_Col{
public:
    Congestion_Col() {}
    Congestion_Col(size_t x , size_t y , size_t z)
    :Col(x), Row(y-1), Layer(z){
        // cong_map.resize( (x-1) * y * z );
        cong_map_3D = vector<vector<vector<double>>> ( x , vector<vector<double>>(y , vector<double>(z,1)) );
    }

    double& operator() (size_t x , size_t y, size_t z ){
        // return cong_map.at(x*Col + y + z * Row * Col );
        return cong_map_3D[x][y][z];
    }

    double& block_to_edge_p(size_t x, size_t y , size_t z){     //positive direction in Col
        if( y >= Row  || y < 0 ){
            cerr<<"wrong positive direction in Vertical"<<endl;
            exit(0);
        }
        // return cong_map.at( z * Row * Col + y + x * Col);
        return cong_map_3D[x][y][z];
    }

    
    double& block_to_edge_n(size_t x, size_t y , size_t z){     //negative direction in Col
        if( y > Row  || y <= 0 ){
            cerr<<"wrong negative direction in Vertical"<<endl;
            exit(0);
        }
        // return cong_map.at( z * Row * Col + y + (x-1) * Col);
        return cong_map_3D[x][y-1][z];
    }

    void print_congestion(){
        cout<<endl;
        cout<<"====================================="<<endl;
        cout<<"congestion Col"<<endl;
        for(size_t i = 0 ; i < Layer ; i++){
            size_t z = 0;
            cout<<"layer : "<<i<<endl<<endl;
            for(size_t k = 0 ; k < Col ; k++){
                cout<<setw(4)<<"Col"<<k;
            }
            cout<<endl<<endl;
            for(size_t j = 0 ; j < Row ; j++){
                for(size_t k = 0; k < Col ; k++){
                    // cout<<setw(5)<<cong_map.at( i * Row * Col + j * Col + k);
                    // cong_map_3D[j][k][i] = z;
                    z++;
                    cout<<setw(5)<<cong_map_3D[k][j][i];
                }
                cout<<endl;
            }
            cout<<endl;
        }
        cout<<"====================================="<<endl;
        cout<<endl;
    }   
private:
    vector<double> cong_map;
    vector<vector<vector<double>>> cong_map_3D;
    size_t Row;
    size_t Col;
    size_t Layer;
};

#endif //CONGESTION_H