#include <iostream>     // std::cout
#include <iterator>     // std::back_inserter
#include <vector>       // std::vector
#include <algorithm>    // std::copy
#include <string>

using namespace std;
void printPattern(vector<string>&);
void insertSquarePattern(string curDir, string initPattern);
void insertSquarePatternHelper(string curDir, string initPattern);
//void modifyMultiRobot(int roboIndx);
string findNextDir(char curDir, int change);

//http://www.cplusplus.com/reference/vector/vector/erase/
//http://www.cplusplus.com/reference/string/string/length/

int numRobot = 4;
int levels = 4;
vector<string> patterns;
string pattern;
	
/*****
 *
 *****/
void insertSquarePattern(string curDir, string initPattern)
{
	pattern = initPattern;
	insertSquarePatternHelper(curDir, pattern);
	patterns.push_back(pattern);
	int j = 0;
	// while (j <= numRobot){

		for (unsigned int i = 0; i <numRobot; i++){
			//call helper
			cout << i;
			// if (numRobot == 2)
			// {
			// 	modifyMultiRobot(pattern,i);
			// }
			//patterns.push_back("H"); //this means it is the end of a pattern robot[i];
		}	
	// }
	
}

void modifyMultiRobot(int roboIndx){
	vector<char> tempPattern;
	copy(pattern.begin(),pattern.end(),back_inserter(tempPattern));

	int curLevel  = 1;
	int curDFirst = (roboIndx*2)+2;
	int curDRestMult  = 2*numBots;
	
	int pastSLevel = 2;
	int curSLevel = 0;
	int totSteps = 0;
	int temptotSteps = 0;

	if(robot == 0){
		while (curLevel <= levels){
			int i = 0;
			temptotSteps = (i*2)+2;
			totSteps += temptotSteps;
			if(curLevel == 2){
				curSLevel = pastSLevel+curDFirst;
			}else{
				curSLevel = pastSLevel+curDRestMult;
			i++;

			for (unsigned int j = 0; j<totSteps; j++){

				
			}
		}
		
		// for(unsigned int i = 0; i< tempPattern.size(); i++){
		// 	if (i > 6){
		// 		currentLevel = 3;
		// 		//repeat partial for i+4;
		// 	}

		}
	}

}

string findNextDir(string curDir, int change){
	string nextDir;
	if(change == 1){ 
		if(curDir == "N"){
			nextDir = "E";
		}else if (curDir =="E"){
			nextDir = "S";
		}else if (curDir =="S"){
			nextDir = "W";
		}else{
			nextDir = "N";
		}
	}
	else{
		nextDir = curDir;
	}
	return nextDir;
}

void insertSquarePatternHelper(string curDir){

	/* Initialize variables starting from level 2 */
	int currentLevel = 2;
	int change = 0;
	int dirCount = 2;
	string nextDir;
	
	/* Operates until pattern is build with the
	   specified number of levels is reached. */
	while (currentLevel <= levels)
	{
		for (int i = 0; i<currentLevel; i++){
			/* Calls helper to find next direction clockwise */
			nextDir = findNextDir(curDir,change);
			/* Insert direction into vector. */
			pattern.append(nextDir);
		}

		curDir = nextDir;
		/* findNextDir internal variable to 
		   stop repeating curDir. */
		change = 1;
		/* Monitor the number of different direction
		   to be 2. This variable determines if a 
		   level is done. */
 		dirCount = dirCount-1;
		
		/* Resets dirCount to 2 and increment currentLevel. */
		if (dirCount == 0) {
			dirCount = 4;
			currentLevel = currentLevel+1;
		}

		/* Allow the direction to change for the next level. */
		if (change == 0){
			change = 1;
		}
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

/*****
 *
 *****/
int main() {

	string initPattern = "NE";
	string initDir = "S";
	insertSquarePattern(initDir,initPattern);
	cout << "Pattern: " << pattern << endl;
	printPattern(patterns);
	
	return 0;
}