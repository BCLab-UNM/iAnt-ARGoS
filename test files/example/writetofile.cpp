#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int main(){

	ofstream outFile;

	outFile.open("sample.txt");

	outFile << "first number : " << 5 << endl;
	outFile.close();

	return 0;
}