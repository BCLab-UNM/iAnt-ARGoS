// back_inserter example
#include <iostream>     // std::cout
#include <iterator>     // std::back_inserter
#include <vector>       // std::vector
#include <algorithm>    // std::copy


using namespace std;
void printPattern(const vector<char>&);
int main () {
  // std::vector<int> foo,bar;
  // for (int i=1; i<=5; i++)
  // { foo.push_back(i); bar.push_back(i*10); }

  // std::copy (bar.begin(),bar.end(),back_inserter(foo));

  // std::cout << "foo contains:";
  // for ( std::vector<int>::iterator it = foo.begin(); it!= foo.end(); ++it )
	 //  std::cout << ' ' << *it;
  // std::cout << '\n';

  // return 0;


  // std::vector<char> foo,bar;
  // std::string str = "hello";

  // foo.push_back('A');
  // foo.push_back('A');
  // foo.push_back('A');
  // foo.push_back('A');
  // foo.push_back('A');

  // std::vector<string> pattern;
  // pattern.push_back("hello");
  // pattern.push_back("NE");
  // //cout << pattern;
  // //printPattern(pattern);
  // std :: vector <char> data = foo;
  // std::copy(str.begin(),str.end(),back_inserter(data));
  // //printPattern(data);
 

  // bar.push_back('B');
  // bar.push_back('B');
  // bar.push_back('B');
  // bar.push_back('B');
  // bar.push_back('B');
  // bar.push_back('B');

  // std::copy (bar.begin(),bar.end(),back_inserter(foo));

  // std::cout << "foo contains:";
  // for ( std::vector<char>::iterator it = foo.begin(); it!= foo.end(); ++it )
	 //  std::cout << ' ' << *it;
  // std::cout << '\n';

  // std::vector<int> x;
  // std::vector<int> y;
  // x.push_back(1);
  // y.push_back(2);

  // for(unsigned int i = 0; i<x.size(); i++)
  // {
  //   x[i] = x[i]+y[i];
  // }
  // printPattern(x);

  vector<char> pattern;
  vector<char> tempPattern;

  pattern.push_back('A');
  pattern.push_back('B');
  pattern.push_back('C');
  pattern.push_back('D');
  pattern.push_back('E');
  pattern.push_back('F');

  copy(pattern.begin(), pattern.end(), back_inserter(tempPattern));

  printPattern(tempPattern);

  tempPattern.clear();

  cout << "tempPattern is cleared" << endl;
  printPattern(tempPattern);



  return 0;
}

 void printPattern(const vector<char>& myPattern){

    cout << "Pattern: ";
    for (unsigned int i = 0; i < myPattern.size(); i++){ 

      // if(myPattern[i] == "hello")
      // { cout<< "north";} 
      cout << myPattern[i] << " ";
    }
    cout << endl;
  }