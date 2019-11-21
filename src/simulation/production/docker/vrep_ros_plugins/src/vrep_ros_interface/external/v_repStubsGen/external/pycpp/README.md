# PyCPP - A (minimalistic) text preprocessor with Python scripting

PyCPP is a text preprocessor with a syntax designed to integrate with C/C++.

It supports execution of python statements, prefixed by `#py`.

## Usage:

```text
$ python pycpp.py --help
usage: pycpp.py [-h] [-i INPUT_FILE] [-o OUTPUT_FILE]
                [-m {tree,python,output}] [-p key=value] [-P path]

optional arguments:
  -h, --help            show this help message and exit
  -i INPUT_FILE, --input-file INPUT_FILE
                        the source file to preprocess, or - for stdin (default: -)
  -o OUTPUT_FILE, --output-file OUTPUT_FILE
                        the output file, or - for stdout (default: -)
  -m {tree,python,output}, --mode {tree,python,output}
                        print output at a specific stage
                        tree: print the internal data structure right after parsing
                        python: print the generated python code before execution
                        output: print the output of the generated python code
  -p key=value, --param key=value
                        set a value that can be read from the template (as pycpp.params["key"])
  -P path, --python-path path
                        additional Python module search path
```

## Template syntax:

Lines beginning with `#py ` will be evaluated by the preprocessor.

Indent is not taken into consideration. Instead, blocks such as `if`, `while`, `for` must end with a corresponding `endif`, `endwhile`, `endfor`. An `if` block can also contain many `elif` and a `else:` part.

Anything else is directly executed in the output script.

Any other line not beginnign with `#py ` will be copied, except parts surrounded by backticks (`\`...\``) which will be evaluated as Python expressions.

## Example:

Input template:

```C++
#include <iostream>

using namespace std;

#py items = ('apple', 'blueberry', 'cherry', 'date', 'entawak')
#py from math import factorial

enum items
{
#py for i, item in enumerate(items):
#py if item == items[-1]:
    `item` = `factorial(i+1)`
#py else:
    `item` = `factorial(i+1)`,
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
```

Resulting output:

```C++
#include <iostream>

using namespace std;


enum items
{
    apple = 1,
    blueberry = 2,
    cherry = 6,
    date = 24,
    entawak = 120
};

int main(int argc, char **argv)
{
    int apple = 1;
    int blueberry = 2;
    int cherry = 6;
    int date = 24;
    int entawak = 120;
    return 0;
}
```

You can also see the intermediate Python script used to generate the output (useful for debugging errors):

```text
$ python pycpp.py --mode python test.cpp
```
```Python
print('#include <iostream>'.format())
print(''.format())
print('using namespace std;'.format())
print(''.format())
items = ('apple', 'blueberry', 'cherry', 'date', 'entawak')
from math import factorial
print(''.format())
print('enum items'.format())
print('{{'.format())
for i, item in enumerate(items):
    if item == items[-1]:
        print('    {} = {}'.format(item, factorial(i+1)))
    else:
        print('    {} = {},'.format(item, factorial(i+1)))
print('}};'.format())
print(''.format())
print('int main(int argc, char **argv)'.format())
print('{{'.format())
for i, item in enumerate(items):
    print('    int {} = {};'.format(item, factorial(i+1)))
print('    return 0;'.format())
print('}}'.format())
print(''.format())
```

