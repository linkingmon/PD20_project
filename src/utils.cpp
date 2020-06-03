#include "utils.h"

#include<iostream>
#include <sstream>
#include <cstdlib>
#include <string>
#include <vector>
using namespace std;


void print_start(string stage_name){
    cout << "=================================================";
    for(int i = 0, end_i = 26 + stage_name.length() / 2 ; i < end_i ; ++i) cout << '\b';
    cout << " " << stage_name << " " << '\n';
}

void print_end(){
    cout << "=================================================" << "\n\n";
}

vector<string> readline2list(istream& is) 
{ 
    string s;
    getline(is, s);

    vector<string> str_list;
    istringstream ss(s); 
  
    do { 
        string word; 
        ss >> word; 
        str_list.push_back(word);
    } while (ss); 
    return str_list;
} 
 
vector<string> split(string s, string delimiter){
    vector<string> str_list;
    size_t pos = 0;
    while ((pos = s.find(delimiter)) != std::string::npos){
        string token = s.substr(0, pos);
        str_list.push_back(token);
        s.erase(0, pos + delimiter.length());
    }
    str_list.push_back(s);
    return str_list;
}