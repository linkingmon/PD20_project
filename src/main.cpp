#include <iostream>
#include <fstream>
#include <string>
#include "placement.h"
#include "placer.h"
#include "router.h"
#include "utils.h"
#include "myUsage.h"
using namespace std;

MyUsage myusage;

int main(int argc, char **argv)
{
    myusage.reset();
    fstream input, output;

    if (argc == 3)
    {
        input.open(argv[1], ios::in);
        output.open(argv[2], ios::out);
        if (!input)
        {
            cerr << "Cannot open the input file \"" << argv[1]
                 << "\". The program will be terminated..." << endl;
            exit(1);
        }
        if (!output)
        {
            cerr << "Cannot open the output file \"" << argv[2]
                 << "\". The program will be terminated..." << endl;
            exit(1);
        }
    }
    else
    {
        cerr << "Usage: ./bin/pr <input filename> <output filename>" << endl;
        exit(1);
    }

    // parsing input
    print_start("Parse input");
    Placement *placement = new Placement(input);
    print_end();
    
    // running placment
    print_start("Run placement");
    Placer *placer = new Placer(placement);
    // placer->place();
    print_end();

    // running routing
    print_start("Run routing");
    Router * router = new Router(placement);
    router->route();
    print_end();

    // write result
    print_start("Write result to file");
    placement->writeResult(output);
    router->writeResult(output);
    print_end();

    return 0;
}
