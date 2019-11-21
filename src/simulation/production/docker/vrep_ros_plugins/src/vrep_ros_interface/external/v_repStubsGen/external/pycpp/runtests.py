from glob import iglob
from sys import argv, exit

verbose=False

for c in argv[1:]:
    exec(c)

def readfile(n, prefix, strip=False):
    filenames = list(iglob('tests/*%s*/%s' % (n, prefix)))
    if len(filenames) < 1:
        return None
    if len(filenames) > 1:
        print('warning: %s matches multiple tests. running %s...' % (name, filenames[0]))
    with open(filenames[0], 'r') as f:
        return {False: lambda x: x, True: lambda x: x.strip()}[strip](f.read())

def runtest(name):
    testdir = list(iglob('tests/*%s*' % name))[0]
    input_str = readfile(name, 'input*')
    if input_str is None:
        raise RuntimeError('test not found: %s' % name)
    if verbose:
        print('RUNNING TEST %s' % name)
    expected_output = readfile(name, 'expected_output*')
    if verbose and expected_output is not None:
        print('TEST HAS EXPECTED OUTPUT')
    expected_exc = readfile(name, 'expected_exception', True)
    if verbose and expected_exc is not None:
        print('TEST HAS EXPECTED EXCEPTION')
    params = {}
    params_s = readfile(name, 'params')
    if params_s is not None:
        for x in params_s.split('\n'):
            if x == '': continue
            if verbose:
                print('SETTING PARAM %s' % x)
            k, v = x.split('=', 1)
            params[k] = v
    import pycpp
    try:
        p = pycpp.PyCPP(input_str)
        p.add_include_path(testdir)
        p.params = params
        output = p.get_output()
        if verbose:
            print(output)
        if expected_exc is not None:
            print('error: test %s failed (was expected to fail with %s)' % (name, expected_exc))
            exit(1)
        if expected_output is not None:
            if output != expected_output:
                print('error: test %s failed (does not match expected output)' % name)
                exit(1)
    except Exception as e:
        en = e.__class__.__name__
        if verbose:
            print('EXCEPTION: %s' % en)
        if expected_exc is None:
            print('error: test %s failed (was not expected to fail with %s)' % (name, type(e)))
            exit(1)
        if en != expected_exc:
            print('error: test %s failed (failed with %s but was expected to fail with %s)' % (name, en, expected_exc))
            exit(1)

for i in iglob('tests/*'):
    i=i.split('/')[1]
    runtest(i)

print('all test passed successfully')
