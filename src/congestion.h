#ifndef CONGESTION_H
#define CONGESTION_H

#include <iostream>

class Congestion{

public:
    Congestion(size_t x, size_t y, size_t z)
    :Row(x) , Col(y) , Layer(z){ 
        cong_map.resize( x * y * z );
    }

    double& operator() (size_t x , size_t y, size_t z ){
        return cong_map.at(x + y * Row + z * Row * Col );
    }
private: 
    vector<double> cong_map;
    size_t Row;
    size_t Col;
    size_t Layer;
};

class Congestion_Row{
public:
    Congestion_Row() {}
    Congestion_Row(size_t x , size_t y , size_t z)
    :Row(x), Col(y-1), Layer(z){
        // cong_map.resize( x * (y-1) * z );
        // vectorM
        cong_map_3D = vector<vector<vector<double>>> ( x , vector<vector<double>>(y-1 , vector<double>(z)) );
    }

    double& operator() (size_t x , size_t y, size_t z ){
        // return cong_map.at(x*Col + y + z * Row * Col );
        return cong_map_3D[x][y][z];
    }

    double& block_to_edge_p(size_t x, size_t y , size_t z){         //positive direction in Row
        if( y >= Col ){
            cerr<<"wrong positive direction in Horizontal"<<endl;
            exit(0);
        }
        // return cong_map.at( z * Row * Col + y * Row + x);
        return cong_map_3D[x][y][z];
    }
    
    double& block_to_edge_n(size_t x, size_t y , size_t z){         //negative direction in Row
        if( y > Col || y <= 0 ){
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
            for(size_t j = 0 ; j < Row ; j++){
                cout<<"Row"<<j<<" ";
                for(size_t k = 0; k < Col ; k++){
                    // cout<<setw(5)<<cong_map.at( i * Row * Col + j * Col + k);
                    cout<<setw(5)<<cong_map_3D[j][k][i];
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

class Congestion_Col{
public:
    Congestion_Col() {}
    Congestion_Col(size_t x , size_t y , size_t z)
    :Row(x-1), Col(y), Layer(z){
        // cong_map.resize( (x-1) * y * z );
        cong_map_3D = vector<vector<vector<double>>> ( x , vector<vector<double>>(y , vector<double>(z)) );
    }

    double& block_to_edge_p(size_t x, size_t y , size_t z){     //positive direction in Col
        if( x >= Row  || x < 0 ){
            cerr<<"wrong positive direction in Vertical"<<endl;
            exit(0);
        }
        // return cong_map.at( z * Row * Col + y + x * Col);
        return cong_map_3D[x][y][z];
    }

    
    double& block_to_edge_n(size_t x, size_t y , size_t z){     //negative direction in Col
        if( x > Row  || x <= 0 ){
            cerr<<"wrong negative direction in Vertical"<<endl;
            exit(0);
        }
        // return cong_map.at( z * Row * Col + y + (x-1) * Col);
        return cong_map_3D[x-1][y][z];
    }

    void print_congestion(){
        cout<<endl;
        cout<<"====================================="<<endl;
        cout<<"congestion Col"<<endl;
        for(size_t i = 0 ; i < Layer ; i++){
            cout<<"layer : "<<i<<endl<<endl;
            for(size_t k = 0 ; k < Col ; k++){
                cout<<setw(4)<<"Col"<<k;
            }
            cout<<endl<<endl;
            for(size_t j = 0 ; j < Row ; j++){
                for(size_t k = 0; k < Col ; k++){
                    // cout<<setw(5)<<cong_map.at( i * Row * Col + j * Col + k);
                    cout<<setw(5)<<cong_map_3D[j][k][i];
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