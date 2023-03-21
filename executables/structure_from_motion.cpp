#include <iostream>
#include <fstream>
#include <string>
#include "utils.cpp"
using namespace std;
// using namespace pr;

int main (int argc, char** argv) {
    ifstream dataset_file("../dataset_and_info/dataset.txt");
    string s;
    for(string line; getline(dataset_file, line); )
    {
        cout << line << endl;
    }
    


}