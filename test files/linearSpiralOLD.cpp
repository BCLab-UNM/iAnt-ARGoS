#include <iostream>
#include <vector>
#include <iterator>
#include <algorithm>    

using namespace std;
int levels;
int run;

void linearSpiral(int levels, int run);

/*Produce the Square Sprial */
void insertSquarePattern(char curDir, vector<char>&, int levels);
char findNextDir(char curDir, int change);

/*Produce the Diamond Spiral */
void printPattern(const vector<char>&);
void diamondPatternHelper(vector<char>&, int levelState, int repeat, int totSteps);
void repeatPartial(vector<char>&,string partial, int repeat);
void insertDiamondPattern(vector<char>&, int levels);

//void show(vector<char> vect);

void insertSquarePattern(char curDir, vector<char>& pattern, int levels)
{
	int currentLevel = 2;
	int change = 0;
	int dirCount = 2;
	char nextDir;
	while (currentLevel <= levels)
	{
		for (int i = 0; i<currentLevel; i++){
			nextDir = findNextDir(curDir,change);
			pattern.push_back(nextDir);
		}

		curDir = nextDir;
		change = 1;
		dirCount = dirCount-1;
		if (dirCount == 0) {
			dirCount = 2;
			currentLevel = currentLevel+1;
		}
		if (change == 0){change = 1;}
	}
}

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

void insertDiamondPattern(vector<char>& pattern, int levels)
{
	int repPart;
	int totSteps;
	int levelmod2;

	vector <char> tempPartial; 
	// repPart = 2;
	// while (repPart <= levels)
	// {
		for (int repPart = 2; repPart<=levels; repPart++){
			totSteps = (repPart*4)+3;
			levelmod2 = repPart%2;
			//cout << "levelmod2" << levelmod2 << '\n';
			cout << "totSteps" << totSteps << '\n';
			// int levelsSquared= levels*levels;
			// totSteps = (2*(levelsSquared)+5*levels+2);
			// cout << "totLSteps line";
			// cout << totSteps << '\n';


			diamondPatternHelper(tempPartial,levelmod2,repPart, totSteps);
			// repPart ++;
		}
		
	// }
	printPattern(pattern);
	printPattern(tempPartial);
	std::copy (tempPartial.begin(),tempPartial.end(),back_inserter(pattern));
	// for (std::vector<char>::iterator it = pattern.begin(); it!= pattern.end(); ++it)
	// {	std::cout << ' ' << *it;
	//   	std::cout << '\n';
	// }
	cout << "final pattern: ";
	cout << "pattern size: ";
	cout << pattern.size() << '\n';
	printPattern(pattern);

}

void diamondPatternHelper(vector<char>& pattern,int levelState, int repeat, int totSteps)
{	
	int state;
	string partial;
	state = 1;

	while (pattern.size() <totSteps)
	{	cout << "totStepshelper ";
		cout << totSteps << '\n';

		cout << "levelState ";
    	cout << levelState << '\n';
		if (state == 5) {state = 1;}
		switch (state){
			case 1:
				if (levelState == 0) 
				{
					pattern.push_back('W');
					//cout << "line 68";
				}
				else
				{
					pattern.push_back('E'); 
					//cout <<"line 74";
				}
			break;

			case 2:
				if (levelState == 0)
				{
					partial = "NE";
					//cout <<"line68 \n";
					repeatPartial(pattern,partial,repeat);
				}
				else {
					partial = "SW";
					//cout <<"line85 \n";
					repeatPartial(pattern,partial,repeat);
				}
			break;
			case 3:
				if (levelState == 0) {
					//cout <<"line 91 \n";
					pattern.push_back('N');
					pattern.push_back('S');
				}
				else {
					//cout <<"line86 \n";
					pattern.push_back('S');
					pattern.push_back('N');
				}
			break;
			case 4:
				if (levelState == 0) {
					partial = "ES";
					//cout <<"line93 \n";
					repeatPartial(pattern,partial,repeat);	
				}
				else {
					partial = "WN";
					//cout <<"line98 \n";
					repeatPartial(pattern,partial,repeat);
				}
			break;
		}
		state++;
		cout << "size: ";
		cout << pattern.size() << '\n';
		// cout << "state: ";
		// cout << state << '\n';
		
	}
}

void repeatPartial(vector <char>& pattern, string partial, int repeat)
{	
	for(int i = 0; i<repeat; i++)
	{
 		std::copy(partial.begin(),partial.end(),back_inserter(pattern));
	}
}

void printPattern(const vector<char>& myPattern){

	cout << "Pattern: ";
	for (unsigned int i = 0; i < myPattern.size(); i++){	
		cout << myPattern[i] << " ";
	}
	cout << endl;
}

// void show(vector<char> vect){

// 	vector<char>::iterator itr;

//   	for(itr=vect.begin(); itr != vect.end(); ++itr){

//     	cout << *itr << " ";
//   	}
//   	cout << endl;
// }

int main ()
{
	// string initDPattern= "NSESWSNWN";
	// vector<char> pattern (initDPattern.begin(), initDPattern.end());
	//insertDiamondPattern(pattern, 3);
	//printPattern(pattern);
	//cout << endl;
	
	char initDir = 'S';
	string initSPattern = "NE";
	vector<char> sPattern (initSPattern.begin(), initSPattern.end());
	int levels = 3;
	insertSquarePattern(initDir,sPattern,levels);
	printPattern(sPattern);

	// vector <vector <int> > position(10,vector <int> (10,0));
	// for (unsigned i = 0; i < position[0].size(); i++){
	// 	for(unsigned j = 0; j<position.size(); j++){

	// 		cout << position[i][j] << " ";
	// 	}
	// }
    
    // vector<char> v;
    // // Declare an iterator to a vector<char>.
    //  	vector<char>::iterator itr;
    
    // // Obtain an iterator to the start of v.
    // itr = v.begin();
    
    // show(pattern);

	//Prompt user to enter pattern
	// cout << "Please enter the number of levels you like to generate \n"; 
	// cout << "followed by which pattarn you like. 1 = Square, 2 = Diamond\n";
	// cin >> levels >> run;
	return 0;
}