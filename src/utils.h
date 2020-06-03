#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <vector>
#include <istream>
using namespace std;

void print_start(string);                   // print the start line of each stage
void print_end();                           // print the end line of each stage
vector<string> readline2list(istream&);     // split by spaces
vector<string> split(string, string);       // split by specific characters

#endif // UTILS_H
