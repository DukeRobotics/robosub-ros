#include <iostream>

#py d = dict(\
    foo=('int', 2),\
    bar=('float', 3),\
    baz=('double', 1)\
)

#py for k,v in d.items():
#py pycpp.include('tmpl', {'name': k, 'type': v[0], 'num_args': v[1]})
#py endfor
