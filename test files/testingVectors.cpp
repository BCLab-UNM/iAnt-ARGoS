#include <iostream>     // std::cout
#include <iterator>     // std::back_inserter
#include <vector>       // std::vector
#include <algorithm>    // std::copy
#include <string>
#include <sstream>

using namespace std;
void printPattern(vector<string>&);
// void insertSquarePattern(vector<string>&, int numRobot, int levels, char curDir);
// void insertSquarePatternHelper(vector<string>&, int numRobot, int levels, char curDir);
void insertSquarePattern(char curDir, string initPattern);
void insertSquarePatternHelper(char curDir, string initPattern, int j);
char findNextDir(char curDir, int change);
string makeRobotString(string pattern, int repeat,);

int numRobot = 4;
int levels = 4;
vector<string> patterns;

// 4 ROBOTS
	
	// R0:
	// 	1 = NX1  EX1
	// 	2 = SX2  WX2
	// 	3 = NX6  EX6
	// 	4 = SX10 WX10

	// R1:
	// 	1 = NX2  EX2
	// 	2 = SX4  WX4
	// 	3 = NX8  EX8
	// 	4 = SX12 WX12

	// R2:
	// 	1 = NX3  EX3
	// 	2 = SX6  WX6
	// 	3 = NX10 EX10
	// 	4 = SX14 WX14

	// R3:
	// 	1 = NX4  EX4
	// 	2 = SX8  WX8
	// 	3 = NX12 EX12
	// 	4 = SX16 WX16
	
/*****
 *
 *****/
void insertSquarePattern(char curDir, string initPattern)
{
	for (unsigned int i = 0; i < numRobot; i++){
		//call helper
		insertSquarePatternHelper(curDir, initPattern, i);
		//patterns.push_back("H"); //this means it is the end of a pattern robot[i];
	}
}

char findNextDir(char curDir, int change){
	char nextDir;
	if(change == 1){
		switch (curDir)
		{
			case 'N':
				nextDir = 'E';
			break;
			case 'E':
				nextDir = 'S';
			break;
			case 'S':
				nextDir = 'W';
			break;
			case 'W':
				nextDir = 'N';
			break;
		}
	}
	else{
		nextDir = curDir;
	}
	return nextDir;
}

void insertSquarePatternHelper(char curDir, string initPattern, int j){

	int currentLevel = 0;
	int change = 0;
	int dirCount = 2;
	int Rincrem;
	char nextDir;
	string tempPattern = initPattern;

		for (unsigned int i = 0; i <currentLevel; i++){
			if(numRobot>1){

				if(currentLevel == 1 && j== 2){
					tempPattern = tempPattern;
				}else if (currentLevel =>1 && j => 2){
					
				}
				
				if(j>1){
					makeRobotString(tempPattern,j+1);
					if(currentLevel == 2 && j == 2){
						tempPattern.append("SSWW");
					}
					if(currentLevel == 3 && j == 2){
						tempPattern.append("NNNNEEEE");
					}
					if(currentLevel == 4 && j == 2){
						tempPatterm.append("SSSSSSWWWWWW");
					}
				}

				
			}
			stringstream ss;
			string s;
			nextDir = findNextDir(curDir,change);
			ss << nextDir;
			ss >> s;
			
			tempPattern.append(s);
		}

		curDir = nextDir;
		dirCount = dirCount-1;

		if(dirCount == 0){
			dirCount = 2;
			currentLevel = currentLevel+1;
		}
		if (change == 0){
			change = 1;
		}

	cout << "tempPattern: " << tempPattern << '\n';
	//add 'H' to end of the pattern string.
}

string makeRobotString(string pattern, int repeat){
	string robotString = pattern;
	for(int i = 0; i <repeat; i++){

	}
}

/*****
 * Helper function that prints the vector <char>.
 *****/
void printPattern(std::vector<string>& myPatterns){

	cout << "Pattern: ";
	for (unsigned int i = 0; i < myPatterns.size(); i++){	
		cout << myPatterns[i] << " ";
	}
	cout << endl;
}
int main() {

	string initPattern = "NE";
	char initDir = 'S';
	insertSquarePattern(initDir,initPattern);


	// string R0 = "NESSWWNNNNNNEEEEEEH";
	// string R1 = "NNEESSSSWWWWNNNNNNNNEEEEEEEESSSSSSSSSSSSWWWWWWWWWWWWH";
	// string R2 = "NNNEEESSSSSSWWWWWWNNNNNNNNNNEEEEEEEEEESSSSSSSSSSSSSSWWWWWWWWWWWWWWH";
	// string R3 = "NNNNEEEESSSSSSSSWWWWWWWWNNNNNNNNNNNNEEEEEEEEEEEESSSSSSSSSSSSSSSSWWWWWWWWWWWWWWWWH";

	// patterns.push_back(R0);
	// patterns.push_back(R1);
	// patterns.push_back(R2);
	// patterns.push_back(R3);

	printPattern(patterns);

	// string apple = "apple";
	// string orange = "orange";
	// string pie = "apple";

	// if (apple.compare(pie) == 0)
	// {
	// 	cout << apple << "==" << pie << endl;
	// }
	// if (apple.compare(orange)!= 0)
	// {
	// 	cout << apple << "!=" << orange << endl;
	// }
	
	return 0;
}