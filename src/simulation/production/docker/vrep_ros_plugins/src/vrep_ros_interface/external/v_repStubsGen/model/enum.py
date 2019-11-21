class Enum(object):
    def __init__(self, plugin, node):
        if node.tag != 'enum':
            raise ValueError('expected <enum>, got <%s>' % node.tag)
        self.plugin = plugin
        self.name = node.attrib['name']
        self.item_prefix = node.attrib.get('item-prefix', '')
        self.base = int(node.attrib.get('base', 0))
        self.items = [type('', (), {
                'name': n.attrib['name'],
                'value': n.attrib.get('value', self.base + i)
            }) for i, n in enumerate(node.findall('item'))]

