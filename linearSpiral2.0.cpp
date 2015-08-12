#include <iostream>
#include <vector>
#include <iterator>
#include <algorithm>    
#include <string>
#include <fstream>

using namespace std;

/*Produce the Square Sprial */
void insertSquarePattern(char curDir, vector<string>&, int levels);
char findNextDir(char curDir, int change);
void printPattern(const vector<string>&, int levels, int robots);

/*Write pattern to text file */
void writeToFile(vector<string>&, int levels);

/*****
 * Inserts N,E,S,W into the vector <char> pattern based on
 * the current level and dirCount. 
 *****/
void insertSquarePattern(char curDir, vector<string>& pattern, int levels){
	
	/* Initialize variables starting from level 2 */
	int currentLevel = 2;
	int change = 0;
	int dirCount = 2;
	char nextDir;
	
	/* Operates until pattern is build with the
	   specified number of levels is reached. */
	while (currentLevel <= levels)
	{
		for (int i = 0; i<currentLevel; i++){
			/* Calls helper to find next direction clockwise */
			nextDir = findNextDir(curDir,change);
			/* Insert direction into vector. */
			pattern.push_back(nextDir);
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
			dirCount = 2;
			currentLevel = currentLevel+1;
		}

		/* Allow the direction to change for the next level. */
		if (change == 0){
			change = 1;
		}
	}

}

/*****
 * Returns the a char that is the next direction
 * current directions changes only if change == 1.
 *****/
char findNextDir (char curDir, int change){
	char newDir;
	if (change == 1)
	{
		switch (curDir)
		{
			case 'N':
				newDir = 'E';
			break;
			case 'E':
				newDir = 'S';
			break;
			case 'S':
				newDir = 'W';
			break;
			case 'W':
				newDir = 'N';
			break;
		}
	}

	else{
		newDir = curDir;
	}
	return newDir;
}

/*****
 * Helper function that prints the vector <char>.
 *****/
void printPattern(const vector<string>& myPattern, int levels){

	cout << "Pattern " << "(" << levels << "): ";
	for (unsigned int i = 0; i < myPattern.size(); i++){	
		cout << myPattern[i] << " ";
	}
	cout << endl;
}

/*****
 * Once done with building the pattern, fucntion is 
 * is called to write pattern as a string to a text file.
 *****/
void writeToFile(vector <char>& pattern, int levels){
	
	ofstream outFile; /* Make an instance of ofstream */

	/* Open a file to write into. */
	outFile.open("SquarePatternOutPut.txt");

	/* Ensure that string is the correct size */
	pattern.resize(pattern.size()); 
	
	for (unsigned int i = 0; i < pattern.size(); i++){
		/* write each element to outFile. */
		outFile << pattern[i]; 
	}

	cout << endl;
	outFile.close();
}

/*****
 * Process and Calls functions that initialize, insert, builds, 
 * prints, and writes pattern to a text file. 
 *****/
int main () 
{
	/* Initialize number of levels, initial direction, pattern. */
	int levels = 30;
	
	// char initDir = 'S';
	// string initSPattern = "NE";

	// /* Converts initSPattern into a vector <char> pattern. */
	// vector<string> pattern (initSPattern.begin(), initSPattern.end());
	
	// /* Call insertSquarePattern to build pattern. */
	// insertSquarePattern(initDir,pattern,levels);
	
	// /* Prints pattern when finished building. */
	// printPattern(pattern, levels);

	// /* Write pattern to a text file. */
	// writeToFile(pattern,levels);	

	return 0;
}