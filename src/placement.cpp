#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <cassert>
#include <vector>
#include <cmath>
#include <map>
#include "cell.h"
#include "net.h"
#include "placement.h"
#include "utils.h"
#include "struc.h"
using namespace std;

void Placement::parseInput(fstream &inFile)
{
    cout << "Start Parsing ..." << '\n';
    vector<string> str_list;
    // max cell move
    str_list = readline2list(inFile);
    _maxMoveCell = stoi(str_list[1]);
    // boundary
    str_list = readline2list(inFile);
    _leftBoundary = stoi(str_list[2]);
    _rightBoundary = stoi(str_list[4]);
    _bottomBoundary = stoi(str_list[1]);
    _topBoundary = stoi(str_list[3]);
    _boundary_width = _rightBoundary - _leftBoundary + 1;
    _boundary_height = _topBoundary - _bottomBoundary + 1;
    // Layers
    str_list = readline2list(inFile);
    _numLayers = stoi(str_list[1]);
    for(int i = 0 ; i < _numLayers ; ++i){
        str_list = readline2list(inFile);
        string name = str_list[1];
        layers.push_back(new Layer(name, str_list[3][0] == 'H', stoi(str_list[4])));
        LayerName2Id[name] = i;
    }
    // Non default supply
    str_list = readline2list(inFile);
    _numNonDefault = stoi(str_list[1]);
    for(int i = 0 ; i < _numNonDefault ; ++i){
        str_list = readline2list(inFile);
        nondefault.push_back(new NonDefault(stoi(str_list[1]), stoi(str_list[0]), stoi(str_list[2]), stoi(str_list[3])));
    }
    // Master Cells
    str_list = readline2list(inFile);
    _numMasterCell = stoi(str_list[1]);
    for(int i = 0 ; i < _numMasterCell ; ++i){
        str_list = readline2list(inFile);
        int numPin = stoi(str_list[2]);
        int numBlockage = stoi(str_list[3]);
        string name = str_list[1];
        Master* cur_ms = new Master(name, numPin, numBlockage , i );
        for(int j = 0 ; j < numPin ; ++j){
            str_list = readline2list(inFile);
            cur_ms->add_pin(str_list[1], LayerName2Id[str_list[2]]);
        }
        for(int j = 0 ; j < numBlockage ; ++j){
            str_list = readline2list(inFile);
            cur_ms->add_blockage(LayerName2Id[str_list[2]], stoi(str_list[3]));
        }
        masters.push_back(cur_ms);
        MasterName2Id[name] = i;
    }
    // Extra demands
    str_list = readline2list(inFile);
    int numExtraDemands = stoi(str_list[1]);
    for(int i = 0 ; i < numExtraDemands ; ++i){
        str_list = readline2list(inFile);
        if(str_list[0][0] == 'a'){
            int mc1 = MasterName2Id[str_list[1]];
            int mc2 = MasterName2Id[str_list[2]];
            int layer_num = LayerName2Id[str_list[3]];
            int cur_demand = stoi(str_list[4]);
            masters[mc1]->add_adj_demand(mc2, layer_num, cur_demand);
            masters[mc2]->add_adj_demand(mc1, layer_num, cur_demand);
        }
        else{
            int mc1 = MasterName2Id[str_list[1]];
            int mc2 = MasterName2Id[str_list[2]];
            int layer_num = LayerName2Id[str_list[3]];
            int cur_demand = stoi(str_list[4]);
            masters[mc1]->add_same_demand(mc2, layer_num, cur_demand);
            masters[mc2]->add_same_demand(mc1, layer_num, cur_demand);
        }
    }
    // Cells
    str_list = readline2list(inFile);
    _numCells = stoi(str_list[1]);
    for(int i = 0 ; i < _numCells ; ++i){
        str_list = readline2list(inFile);
        string cell_name = str_list[1];
        _cellArray.push_back(new Cell(cell_name, masters[MasterName2Id[str_list[2]]], stoi(str_list[3]), stoi(str_list[4]), str_list[5] == "Movable", i));
        _CellName2Id[cell_name] = i;
    }
    // Nets
    str_list = readline2list(inFile);
    _numNets = stoi(str_list[1]);
    for(int i = 0 ; i < _numNets ; ++i){
        str_list = readline2list(inFile);
        string net_name = str_list[1];
        int num_pin = stoi(str_list[2]);
        int min_layer = (str_list[3] == "NoCstr") ? -1 : LayerName2Id[str_list[3]];
        Net * cur_net = new Net(net_name, num_pin, min_layer);
        for(int j = 0 ; j < num_pin ; ++j){
            str_list = readline2list(inFile);
            str_list = split(str_list[1], "/");
            Pin* cur_pin = _cellArray[_CellName2Id[str_list[0]]]->getPin(str_list[1]);
            cur_net->addPin(cur_pin);
            cur_pin->addNetId(i);
        }
        _netArray.push_back(cur_net);
    }
    // Routes
    str_list = readline2list(inFile);
    int num_route = stoi(str_list[1]);
    cout << "Num of routes: " << num_route << endl;

    cout << "Done Parsing ..." << '\n';
    printSummary();
    return;
}


void Placement::printSummary() const
{
    cout << endl;
    cout << "==================== Summary ====================" << endl;
    cout << "* Max Cell Move: " << _maxMoveCell << '\n';
    cout << "* Boundary: (" << _leftBoundary << "," << _bottomBoundary << ") : (" << _bottomBoundary << "," << _topBoundary << ")\n";
    cout << "* Num of Layers: " << _numLayers << '\n'; 
    for(int i = 0 ; i < _numLayers ; ++i) layers[i]->print();
    cout << "* Num of Non default supply grid: " << _numNonDefault << '\n';
    for(int i = 0 ; i < _numNonDefault ; ++i) nondefault[i]->print();
    cout << "* Num of Master Cells: " << _numMasterCell << '\n';
    for(int i = 0 ; i < _numMasterCell ; ++i) masters[i]->print();
    cout << "* Num of Cells: " << _numCells << '\n';
    for(int i = 0 ; i < _numCells ; ++i) _cellArray[i]->print();
    cout << "* Num of Cells: " << _numNets << '\n';
    for(int i = 0 ; i < _numNets ; ++i) _netArray[i]->print();
    cout << "=================================================" << endl;
    cout << endl;
    return;
}

void Placement::reportNet() const
{
    // cout << "Number of nets: " << _netNum << endl;
    // for (size_t i = 0, end_i = _netArray.size(); i < end_i; ++i)
    // {
    //     cout << setw(8) << _netArray[i]->getName() << ": ";
    //     vector<int> cellList = _netArray[i]->getCellList();
    //     for (size_t j = 0, end_j = cellList.size(); j < end_j; ++j)
    //     {
    //         cout << setw(8) << _cellArray[cellList[j]]->getName() << " ";
    //     }
    //     cout << endl;
    // }
    return;
}

void Placement::reportCell() const
{
    cout << "* Num of Cells: " << _numCells << '\n';
    for(int i = 0 ; i < _numCells ; ++i) _cellArray[i]->print();
    return;
}

void Placement::writeResult(fstream &outFile)
{
    // stringstream buff;
    // buff << _cutSize;
    // outFile << "Cutsize = " << buff.str() << '\n';
    // buff.str("");
    // buff << _partSize[0];
    // outFile << "G1 " << buff.str() << '\n';
    // for (size_t i = 0, end = _cellArray.size(); i < end; ++i)
    // {
    //     if (_cellArray[i]->getPart() == 0)
    //     {
    //         outFile << _cellArray[i]->getName() << " ";
    //     }
    // }
    // outFile << ";\n";
    // buff.str("");
    // buff << _partSize[1];
    // outFile << "G2 " << buff.str() << '\n';
    // for (size_t i = 0, end = _cellArray.size(); i < end; ++i)
    // {
    //     if (_cellArray[i]->getPart() == 1)
    //     {
    //         outFile << _cellArray[i]->getName() << " ";
    //     }
    // }
    // outFile << ";\n";
    // return;
}

void Placement::clear()
{
    // for (size_t i = 0, end = _cellArray.size(); i < end; ++i)
    // {
    //     delete _cellArray[i];
    // }
    // for (size_t i = 0, end = _netArray.size(); i < end; ++i)
    // {
    //     delete _netArray[i];
    // }
    return;
}
