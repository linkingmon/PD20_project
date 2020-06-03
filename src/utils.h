#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <vector>
#include <istream>
using namespace std;

void print_start(string);
void print_end();
vector<string> readline2list(istream&);
vector<string> split(string, string);

#endif // UTILS_H
