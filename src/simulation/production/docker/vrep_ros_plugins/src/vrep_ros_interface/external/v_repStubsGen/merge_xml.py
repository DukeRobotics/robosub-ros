import xml.etree.ElementTree as ET
from sys import argv, exit

if len(argv) != 4:
    print('usage: {} <xml1> <xml2> <output>'.format(argv[0]))
    exit(1)

with open(argv[1], 'rb') as f1, open(argv[2], 'rb') as f2, open(argv[3], 'wb') as f3:
    tree = list(map(ET.parse, (f1, f2)))
    root = list(map(lambda tree: tree.getroot(), tree))
    for e in root[1]:
        root[0].append(e)
    tree[0].write(f3, encoding='utf-8', xml_declaration=True)

