// basic file operations
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iterator>
#include <algorithm>

using namespace std;

int main () {
    
    ifstream infile;
    infile.open("SquarePatternOutPut.txt");
    
    //Check For Error
    if (infile.fail()){
        cerr << "Error Opening File.";
        exit(1);
    }
    
//    int x,y;
//    
//    infile >> x >> y;
//    
//    cout << "num 1: " << x << endl;
//    cout << "num 2: " << y << endl;
    
    //Read file until reach the end
    string line;
    if (infile.is_open())
    {
        while (getline (infile,line)){
            cout << line << '\n';
        }
        infile.close();
    }
    return 0;
}