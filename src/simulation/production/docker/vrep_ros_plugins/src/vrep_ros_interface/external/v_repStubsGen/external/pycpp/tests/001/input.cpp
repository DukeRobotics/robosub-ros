#include <iostream>

using namespace std;

#py items = ('apple', 'blueberry', 'cherry', 'date', 'entawak')
#py from math import factorial

enum items
{
#py for i, item in enumerate(items):
#py if i == 0:
    `item` = `factorial(i+1)`, // first
#py elif i < len(items) - 1:
    `item` = `factorial(i+1)`,
#py else:
    `item` = `factorial(i+1)`
#py endif
#py endfor
};

int main(int argc, char **argv)
{
#py for i, item in enumerate(items):
    int `item` = `factorial(i+1)`;
#py endfor
    return 0;
}

