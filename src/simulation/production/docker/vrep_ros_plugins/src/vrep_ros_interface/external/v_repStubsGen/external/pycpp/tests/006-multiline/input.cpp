#include <iostream>

// the next line will not be continued by pycpp because doesn't start with #py
#define TYPE \
    int

// the following 4 lines will be fed to python in 1 shot
#py x = { \
    'a': 100, \
    'b': 200 \
}

// testcase implementation 1
#py z = '''\
#py raise RuntimeError('failed subcase 1') \
#py '''

// testcase implementation 2
#py pass2=False
\
#py pass2=True
#py if not pass2:
#py raise RuntimeError('failed subcase 2')
#py endif

#py def f(x):\
    y = 0\
    while x:\
        y += x\
        x = int(x/2)\
    return y

using namespace std;

int main(int argc, char **argv)
{
#py for k, v in x.items():
    int `k` = `f(v)`;
#py endfor
    return 0;
}

