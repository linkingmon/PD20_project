/****************************************************************************
  FileName     [ myMinHeap.h ]
  PackageName  [ util ]
  Synopsis     [ Define MinHeap ADT ]
  Author       [ Chung-Yang (Ric) Huang ]
  Copyright    [ Copyleft(c) 2014-present LaDs(III), GIEE, NTU, Taiwan ]
****************************************************************************/

#ifndef MIN_HEAP_H
#define MIN_HEAP_H

#include <algorithm>
#include <vector>

template <class Data>
class MinHeap
{
public:
    MinHeap(size_t s = 0) { if (s != 0) _data.reserve(s); }
    MinHeap(vector<Data>& a):_data(a){ build_min_heap();}
    ~MinHeap() {}

    void clear() { _data.clear(); }

    // For the following member functions,
    // We don't respond for the case vector "_data" is empty!
    const Data& operator [] (size_t i) const { return _data[i]; }   
    Data& operator [] (size_t i) { return _data[i]; }

    size_t size() const { return _data.size(); }
    bool empty() const { return (_data.size() == 0 ); }
    // TODO
    const Data& min() const { return _data[0]; }
    void build_min_heap(){
        for(int  i = size()/2 - 1 ; i >= 0 ; i-- ){
            MinHeapify(i);     
        }
    }
    void print_heap(){
        
        int i ;
        cout<<endl;
        cout<<"===================Heap====================="<<endl;
        cout<<"heap size: "<<size()<<endl;
        int _size = size();
        for (i = 1 ; i <= _size/2 - 1 ; i++){
            cout<<"index "<< i-1 <<" weight:" << _data[i-1] << ", left child: "<< _data[2*i-1]<<", right child: "<<_data[2*i]<<endl;
        }
        cout<<"index "<< i-1 <<" weight:"<< _data[i-1] << ", left child: "<< _data[2*i-1];
        if(2*i < size())
            cout<<", right child: "<<_data[2*i];
        cout<<endl;
        cout<<"============================================"<<endl;
        cout<<endl;
    }
    void insert(Data& d) {
        int p , t =  ( size()+1 )  ;
        //_data.push_back(Data("0",0));
        _data.push_back (d) ;
        while( t > 1 ) {
            p =  t/2 ; 
            if (  _data[p-1] < d ) 
            break;
            _data[t-1] = _data[p-1] ;   
            update_position(t-1);
            
            t = p ;
        }
        _data[t-1] = d ;
        update_position(t-1);
    }
    void MinHeapify(int node){
        int n = node + 1;   
        int left = 2*n  ,          // 取得left child
            right = 2*n + 1 ,     // 取得right child
            smallest;               // smallest用來記錄包含root與child, 三者之中Key最小的node

        if (left <= size() && _data[left - 1] < _data[node] )
            smallest = left - 1;
        else
            smallest = node;

        if (right <= size() && _data[right - 1] < _data[node] )
            smallest = right - 1;

        if (smallest != node) {                 // 如果目前node的Key不是三者中的最小
            swap(_data[smallest], _data[node]);   // 就調換node與三者中Key最小的node之位置
            update_position(smallest);
            update_position(node);
            MinHeapify(smallest);       // 調整新的subtree成Min Heap
        }
    }

    void delMin() { delData ( 0 ) ; }

    Data ExtractMin() {
        Data a = _data[0];
        delMin();
        return a;
    }
    void delData(size_t i) { 
            
        int p = i + 1 ;
        int t = p * 2 ;

        while ( t <= size() ){
            if ( t < size() ) //has right child
                if ( _data[t] < _data[t-1] )    
                    ++t;//to the smaller child
            if( _data[ size() -1 ]  < _data [t-1] )
                break;
            _data[p-1] =  _data [t-1] ;
            update_position(p-1);
            p = t ;
            t = p * 2 ;
        }
        _data[p-1] = _data[ size() -1 ] ;
        update_position(p-1);
        _data.pop_back();

    }

    int find_position( Data& a ){
        // for(size_t i = 0 ; i < size(); i++){
        //     if( _data[i] == a )
        //         return i;
        // }
        return (a.index);
    }

    void Decrease_key( Data a ){
        int idx = find_position(a);
        Decrease_Key( idx , a);
    }   

    void update_position( int idx){
        _data[idx].update_position(idx);
    }

    void Decrease_Key( int idx , Data a){
        Data original_a = _data[idx];
        if( _data[idx] < a ){
            return ; 
        }
        _data[idx] = a;
        int p , t =  idx + 1  ;
        while( t > 1 ) {
            p =  t/2 ; 
            if (  _data[p-1] < a ) 
            break;
            _data[t-1] = _data[p-1] ;   
            update_position(t-1);
            t = p ;
        }
        _data[t-1] = a ;
        update_position(t-1);
    }

    private:
    // DO NOT add or change data members
    vector<Data>   _data;
};

template <class Data>
class MinHeap_pointer
{
public:
    MinHeap_pointer(size_t s = 0) { _data.clear(); if (s != 0) _data.reserve(s); }
    MinHeap_pointer(vector<Data*> &a):_data(a){ build_min_heap();}
    ~MinHeap_pointer() {}

    void clear() { _data.clear(); }

    // For the following member functions,
    // We don't respond for the case vector "_data" is empty!
    const Data& operator [] (size_t i) const { return _data[i]; }   
    Data& operator [] (size_t i) { return _data[i]; }

    size_t size() const { return _data.size(); }
    bool empty() const { return (_data.size() == 0 ); }
    // TODO
    const Data* min() const { return _data[0]; }
    void build_min_heap(){
        for(int  i = size()/2 - 1 ; i >= 0 ; i-- ){
            MinHeapify(i);     
        }
    }
    void print_heap(){
        
        int i ;
        cout<<endl;
        cout<<"===================Heap====================="<<endl;
        cout<<"heap size: "<<size()<<endl;
        int _size = size();
        for (i = 1 ; i <= _size/2 - 1 ; i++){
            cout<<"index "<< i-1 <<" weight:" << (* _data[i-1] )<< ", left child: "<< (*_data[2*i-1]) <<", right child: "<< (*_data[2*i])<<endl;
        }
        cout<<"index "<< i-1 <<" weight:"<< (*_data[i-1] )<< ", left child: "<<(* _data[2*i-1] );
        if(2*i < size())
            cout<<", right child: "<<_data[2*i];
        cout<<endl;
        cout<<"============================================"<<endl;
        cout<<endl;
    }
    void insert(Data* d) {
        int p , t =  ( size()+1 )  ;
        //_data.push_back(Data("0",0));
        _data.push_back (d) ;
        while( t > 1 ) {
            p =  t/2 ; 
            if (  (*_data[p-1]) < (*d) ) 
            break;
            _data[t-1] = _data[p-1] ;   
            update_position(t-1);
            
            t = p ;
        }
        // (*_data[t-1]) = (*d) ;
        _data[t-1] = d;
        update_position(t-1);
    }
    void MinHeapify(int node){
        int n = node + 1;   
        int left = 2*n  ,          // 取得left child
            right = 2*n + 1 ,     // 取得right child
            smallest;               // smallest用來記錄包含root與child, 三者之中Key最小的node

        if (left <= size() && (*_data[left - 1]) < (*_data[node]) )
            smallest = left - 1;
        else
            smallest = node;

        if (right <= size() && (*_data[right - 1]) < (*_data[smallest]) )
            smallest = right - 1;

        if (smallest != node) {                 // 如果目前node的Key不是三者中的最小
            swap(_data[smallest], _data[node]);   // 就調換node與三者中Key最小的node之位置
            update_position(smallest);
            update_position(node);
            MinHeapify(smallest);       // 調整新的subtree成Min Heap
        }
    }

    void delMin() { delData ( 0 ) ; }

    Data* ExtractMin() {
        Data* a = _data[0];
        delMin();
        return a;
    }
    void delData(size_t i) { 
            
        int p = i + 1 ;
        int t = p * 2 ;

        while ( t <= size() ){
            if ( t < size() ) //has right child
                if ( (*_data[t]) < (*_data[t-1]) )    
                    ++t;//to the smaller child
            if( (*_data[ size() -1 ] ) < (*_data [t-1]) )
                break;
            _data[p-1] =  _data [t-1] ;
            update_position(p-1);
            p = t ;
            t = p * 2 ;
        }
        _data[p-1] = _data[ size() -1 ] ;
        update_position(p-1);
        _data.pop_back();

    }

    int find_position( Data* a ){
        // for(size_t i = 0 ; i < size(); i++){
        //     if( _data[i] == a )
        //         return i;
        // }
        return (a->index);
    }

    void Decrease_key( Data* a ){
        int idx = find_position(a);
        Decrease_Key( idx , a);
    }   

    void update_position( int idx){
        _data[idx]->update_position(idx);
    }

    void Decrease_Key( int idx , Data* a){
        Data* original_a = _data[idx];
        if( (*_data[idx]) < (*a) ){
            return ; 
        }
        _data[idx] = a;
        int p , t =  idx + 1  ;
        while( t > 1 ) {
            p =  t/2 ; 
            if (  (*_data[p-1]) < (*a) ) 
            break;
            _data[t-1] = _data[p-1] ;   
            update_position(t-1);
            t = p ;
        }
        _data[t-1] = a ;
        update_position(t-1);
    }

    private:
    // DO NOT add or change data members
    vector<Data*>   _data;
};


#endif // MY_MIN_HEAP_H
