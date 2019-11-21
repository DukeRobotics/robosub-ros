from sys import argv, exit, stderr
import os
import re
import rospkg
import rosmsg

class TypeSpec:
    '''
    parse a type specification, such as Header, geometry_msgs/Point, or string[12]
    '''

    fast_write_types = {'int32': 'Int32', 'float32': 'Float', 'float64': 'Double'}

    ctype_builtin = {
            'bool':         'uint8_t',
            'int8':         'int8_t',
            'uint8':        'uint8_t',
            'int16':        'int16_t',
            'uint16':       'uint16_t',
            'int32':        'int32_t',
            'uint32':       'uint32_t',
            'int64':        'int64_t',
            'uint64':       'uint64_t',
            'float32':      'float',
            'float64':      'double',
            'string':       'std::string',
            'time':         'ros::Time',
            'duration':     'ros::Duration'
    }

    deprecated_builtins = {
            'byte':         'int8',
            'char':         'uint8'
    }

    def __init__(self, s):
        def is_identifier(s):
            return re.match('^[a-zA-Z_][a-zA-Z0-9_]*$', s)
        self.array = False
        self.array_size = None
        m = re.match(r'^(.*)\[(\d*)\]$', s)
        if m:
            self.array = True
            s = m.group(1)
            if len(m.group(2)) > 0:
                self.array_size = int(m.group(2))
        # perform substitutions:
        if s in self.deprecated_builtins: s = self.deprecated_builtins[s]
        # check builtins:
        self.builtin = s in self.ctype_builtin
        self.fullname = s
        if self.builtin:
            self.mtype = s
        else:
            if '/' not in s:
                raise ValueError('bad type: %s' % s)
            tok = s.split('/')
            if len(tok) != 2:
                raise ValueError('bad type: %s' % s)
            if not is_identifier(tok[0]) or not is_identifier(tok[1]):
                raise ValueError('bad type: %s' % s)
            self.package = tok[0]
            self.mtype = tok[1]

    # normalize fullname to C identifier (replace / with __)
    def normalized(self):
        return ('{}__'.format(self.package) if not self.builtin else '') + self.mtype

    # get C++ type declaration
    def ctype(self):
        if self.builtin: return self.ctype_builtin[self.mtype]
        return self.package + '::' + self.mtype

    def __str__(self):
        t = self.mtype
        if not self.builtin:
            t = self.package + '/' + t
        if self.array:
            t += '[]'
        return t

class Fields:
    def __init__(self, lines):
        # parse msg / srv definition
        self.fields = {}

        for ln in lines:
            if ln.startswith('  '):
                # ignore expansions of nested types
                continue

            if ln == '':
                # ignore empty lines
                continue

            ln_orig1 = ln

            ln = ln.replace('=',' = ')

            tokens = ln.split()

            if len(tokens) == 4 and tokens[2] == '=':
                # it's a constant definition: ignore
                continue

            if len(tokens) == 2:
                t = TypeSpec(tokens[0])
                n = tokens[1]
                self.fields[n] = t
            else:
                raise ValueError('unrecognized line: ' + ln_orig1)

class MsgInfo:
    def __init__(self, msg_name, rospack=None):
        # parse msg definition
        lines = rosmsg.get_msg_text(msg_name, False, rospack).splitlines()
        self.typespec = TypeSpec(msg_name)
        msg = Fields(lines)
        self.fields = msg.fields

class SrvInfo:
    def __init__(self, srv_name, rospack=None):
        # parse srv definition
        lines = rosmsg.get_srv_text(srv_name, False, rospack).splitlines()
        sep = '---'
        if sep not in lines:
            raise ValueError('bad srv definition')
        i = lines.index(sep)
        self.typespec = TypeSpec(srv_name)
        self.req = Fields(lines[:i])
        self.resp = Fields(lines[i+1:])

def get_msgs_info(messages_file):
    # Utility class used by rosmsg functions
    # Initializing it in advance yields better performance
    rospack = rospkg.RosPack()

    # populate msg list
    msg_list = set()
    with open(messages_file) as f:
        for l in f.readlines():
            l = l.strip()
            if not l: continue
            msg_list.add(l)

    # get msg definitions
    msg_fields = {}
    for msg in sorted(msg_list):
        try:
            msg_fields[msg] = MsgInfo(msg, rospack)
        except rosmsg.ROSMsgException:
            print('WARNING: bad msg: %s' % msg)
            continue

    return msg_fields

def get_srvs_info(services_file):
    # Utility class used by rosmsg functions
    # Initializing it in advance yields better performance
    rospack = rospkg.RosPack()

    # populate srv list
    srv_list = set()
    with open(services_file) as f:
        for l in f.readlines():
            l = l.strip()
            if not l: continue
            srv_list.add(l)

    # get srv definitions
    srv_fields = {}
    for srv in sorted(srv_list):
        try:
            srv_fields[srv] = SrvInfo(srv, rospack)
        except rosmsg.ROSMsgException:
            print('WARNING: bad srv: %s' % srv)
            continue

    return srv_fields

def get_msgs_srvs_info(messages_file, services_file):
    # Utility class used by rosmsg functions
    # Initializing it in advance yields better performance
    rospack = rospkg.RosPack()

    msg_fields = get_msgs_info(messages_file)
    srv_fields = get_srvs_info(services_file)
    for srv, info in srv_fields.items():
        for k, v in {'Request': info.req.fields, 'Response': info.resp.fields}.items():
            msg = srv + k
            try:
                msg_fields[msg] = type('', (), dict(typespec=TypeSpec(msg), fields=v))
            except rosmsg.ROSMsgException:
                print('WARNING: bad msg: %s' % srv)
                continue

    return msg_fields

