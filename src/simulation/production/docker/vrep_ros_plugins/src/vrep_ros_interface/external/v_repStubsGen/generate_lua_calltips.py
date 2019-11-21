from sys import argv, exit
import re

if len(argv) != 5:
    print('usage: {} <long-plugin-name> <short-plugin-name> <lua-file> <output-cpp-file>'.format(argv[0]))
    exit(1)

longPlugName = argv[1]
shortPlugName = argv[2]
luafile = argv[3]
outfile = argv[4]

fun = None
args, rets = [], []

with open(outfile, 'w') as fout:
    def output():
        if fun:
            f, fdesc = fun
            calltip = (','.join(x[0]+' '+x[1] for x in rets)+('=' if rets else '')+'sim'+shortPlugName+'.'+f+'('+','.join(x[0]+' '+x[1] for x in args)+')')
            if fdesc:
                calltip += '\\n\\n' + fdesc
            while calltip[-2:] == '\\n': calltip = calltip[:-2]
            fout.write('simRegisterScriptCallbackFunctionE("sim{}.{}@{}", "{}", NULL);\n'.format(shortPlugName, f, longPlugName, calltip))

    with open(luafile, 'r') as f:
        for lineno, line in enumerate(f):
            lineno += 1
            m1 = re.match(r'\s*--\s*@(\w+)\b(.*)$', line)
            if m1:
                key = m1.group(1)
                rest = m1.group(2).strip()
                if key == 'fun':
                    m2 = re.match(r'(\w+)(|\s+.*)$', rest)
                    if m2:
                        fun = (m2.group(1), m2.group(2))
                    else:
                        print('%s:%d: bad arguments. must be: @fun <funcName> [description]' % (luafile, lineno))
                        exit(2)
                elif key in ('arg', 'ret'):
                    m2 = re.match(r'(\w+)\s+(\w+)\s+(.*)$', rest)
                    if m2:
                        item = (m2.group(1), m2.group(2), m2.group(3))
                        (rets if key == 'ret' else args).append(item)
                    else:
                        print('%s:%d: bad arguments. must be: @%s <type> <name> <description>' % (luafile, lineno, key))
                        exit(2)
            else:
                output()
                fun = None
                args, rets = [], []
        output()

