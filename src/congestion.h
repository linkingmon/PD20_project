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
        cong_map.resize( x * (y-1) * z );
    }

    double& block_to_edge(size_t x, size_t y , size_t z){
        if( y >= Col ){
            cerr<<"wrong direction in Horizontal"<<endl;
            exit(0);
        }
        return cong_map.at( z * Row * Col + y * Row + x);
    }
private:
    vector<double> cong_map;
    size_t Row;
    size_t Col;
    size_t Layer;
};

class Congestion_Col{
public:
    Congestion_Col() {}
    Congestion_Col(size_t x , size_t y , size_t z)
    :Row(x-1), Col(y), Layer(z){
        cong_map.resize( (x-1) * y * z );
    }

    double& block_to_edge(size_t x, size_t y , size_t z){
        if( x >= Row ){
            cerr<<"wrong direction in Vertical"<<endl;
            exit(0);
        }
        return cong_map.at( z * Row * Col + y + x * Col);
    }
private:
    vector<double> cong_map;
    size_t Row;
    size_t Col;
    size_t Layer;
};

#endif