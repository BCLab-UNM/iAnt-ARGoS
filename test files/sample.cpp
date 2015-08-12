#include <iostream>
#include <vector>

using namespace std;
//myVector.push_back()
//myVector.at(index)
//myVector.size()
//myVector.begin(()
//myVector.insert(myVector.begin()+integer,newValue); add element b4 index
//myVector.erase(myVector.being()+integer) remove at specific number
//myVector.clear()
//myVector.empty() returns bool
void insertPattern(vector<int>&);
void printVector (const vector<int>&);
void reverse(const vector<int>&);
void printEvens(const vector<int>&);
void replace (vector<int>&);

int main ()
{
	vector<int> myVector;

	// myVector.push_back(3);
	// myVector.push_back(7);
	// myVector.push_back(4);
	// myVector.push_back(9);

	// cout << "Vector: ";

	// for (unsigned int i = 0; i < myVector.size(); i++) {

	// 	cout << myVector[i] << " ";
	// }

	// myVector.insert(myVector.begin() + );

	// cout << endl;
	insertPattern(myVector);
	printVector(myVector);
	reverse(myVector);
	printEvens(myVector);
	replace(myVector);
	printVector(myVector);
	return 0;
}

void replace(vector<int>& newMyVector)
{
	int old, replace;
	cout << "Type in a number to be replaced with another number: ";
	cin >> old >> replace;

	for (unsigned int i = 0; i< newMyVector.size(); i++){
		if (newMyVector[i]==old)
		{
			newMyVector[i] = replace;
		}
	}
	printVector(newMyVector);
	cout << endl;
}

void printEvens(const vector<int>& newMyVector)
{	
	cout << "Evens: ";
	for (unsigned int i = 0; i < newMyVector.size(); i++)
	{
		if (newMyVector[i]%2==0)
		{
			cout << newMyVector[i] << " ";
		}
	}
	cout << endl;
}

void reverse(const vector<int>& newMyVector) {
	cout << "Vector Reverse: ";
	for (unsigned int i = newMyVector.size(); i>0; i--){
		cout << newMyVector[i-1] << " ";
	}
	cout << endl;
}

void printVector (const vector<int>& newMyVector)
{
	cout << "Vector: ";

	for (unsigned int i = 0; i <newMyVector.size(); i++){
		cout << newMyVector[i] << " ";
	}
	cout << endl;
}

void insertPattern(vector<int>& newMyVector)
{
	cout << "Type in a list of numbers (-1 to stop): ";
	int input;
	cin >> input;
	while (input != -1) {
		newMyVector.push_back(input);
		cin >> input;
	}
	cout << endl;
}