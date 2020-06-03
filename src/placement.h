#ifndef PLACEMENT_H
#define PLACEMENT_H

#include <fstream>
#include <vector>
#include <map>
#include "cell.h"
#include "net.h"
#include "struc.h"
using namespace std;

class Placement
{
public:
    // constructor and destructor
    Placement(fstream &inFile)
    {
        parseInput(inFile);
    }
    ~Placement()
    {
        clear();
    }

    // modify method
    void parseInput(fstream &inFile);

    // member functions about reporting
    void printSummary() const;
    void reportNet() const;
    void reportCell() const;
    void writeResult(fstream &outFile);

private:
    // maximum move cell
    int _maxMoveCell;
    // boundary of the placement
    int _leftBoundary;
    int _rightBoundary;
    int _bottomBoundary;
    int _topBoundary;
    // Layer charatersitics
    int _numLayers;
    vector<Layer*> layers; 
    map<string, int> LayerName2Id;
    // supply related
    int _numNonDefault;
    vector<NonDefault*> nondefault;
    // master cells
    int _numMasterCell;
    vector<Master*> masters;
    map<string, int> MasterName2Id;
    // cells
    int _numCells;
    vector<Cell*> _cellArray;
    map<string, int> _CellName2Id;
    // nets
    int _numNets;
    vector<Net*> _netArray;

    // Clean up Placement
    void clear();
};

#endif // PLACEMENT_H
