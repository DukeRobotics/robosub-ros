print("hello world")

line = "36087D002D00B300800080008000800000000000000000FF0080008000800080000000000F013A2F550000FF05030A"
print(len(line))

byte_string = bytearray.fromhex(line)

print((int("007D", 16)))

print(byte_string)

DATA_ID = int.from_bytes(byte_string[0:2], byteorder='little')
DATA_STRUCT = byte_string[2:4]
NUM_BYTES = int.from_bytes(byte_string[4:8], byteorder='little')
SYS_CONFIG = byte_string[8:10]
X_VEL = int.from_bytes(byte_string[10:14], byteorder='little', signed=True)
Y_VEL = int.from_bytes(byte_string[14:18], byteorder='little', signed=True)
Z_VEL = int.from_bytes(byte_string[18:22], byteorder='little', signed=True)
E_VEL = int.from_bytes(byte_string[22:26], byteorder='little', signed=True)
BM1 = int.from_bytes(byte_string[26:30], byteorder='little', signed=False)
BM2 = int.from_bytes(byte_string[30:34], byteorder='little', signed=False)
BM3 = int.from_bytes(byte_string[34:38], byteorder='little', signed=False)
BM4 = int.from_bytes(byte_string[38:42], byteorder='little', signed=False)
BOTTOM_STATUS = byte_string[42:44]
VEL_1 = int.from_bytes(byte_string[44:48], byteorder='little', signed=True)
VEL_2 = int.from_bytes(byte_string[48:52], byteorder='little', signed=True)
VEL_3 = int.from_bytes(byte_string[52:56], byteorder='little', signed=True)
VEL_4 = int.from_bytes(byte_string[56:60], byteorder='little', signed=True)
REF_LAYER_START = int.from_bytes(byte_string[60:64], byteorder='little', signed=False)
REF_LAYER_END = int.from_bytes(byte_string[64:68], byteorder='little', signed=False)
REF_LAYER_STATUS = byte_string[68:70]
TOFP_HOUR = int.from_bytes(byte_string[70:72], byteorder='little', signed=False)
TOFP_MIN = int.from_bytes(byte_string[72:74], byteorder='little', signed=False)
TOFP_SEC = int.from_bytes(byte_string[74:76], byteorder='little', signed=False)
TOFP_HUNDRETH = int.from_bytes(byte_string[76:78], byteorder='little', signed=False)
LEAK_SENSOR = byte_string[78:82]
SPEED_OF_SOUND = int.from_bytes(byte_string[82:86], byteorder='little', signed=False)
TEMPERATURE = int.from_bytes(byte_string[86:90], byteorder='little', signed=False)
CHECKSUM = int.from_bytes(byte_string[90:94], byteorder='little', signed=False)

print(f"DATA_ID: {DATA_ID}, DATA_STRUCT: {DATA_STRUCT}, NUM_BYTES: {NUM_BYTES}, SYS_CONFIG: {SYS_CONFIG}, X_VEL: {X_VEL}, Y_VEL: {Y_VEL}, Z_VEL: {Z_VEL}, E_VEL: {E_VEL}, BM1: {BM1}, BM2: {BM2}, BM3: {BM3}, BM4: {BM4}, BOTTOM_STATUS: {BOTTOM_STATUS}, VEL_1: {VEL_1}, VEL_2: {VEL_2}, VEL_3: {VEL_3}, VEL_4: {VEL_4}, REF_LAYER_START: {REF_LAYER_START}, REF_LAYER_END: {REF_LAYER_END}, REF_LAYER_STATUS: {REF_LAYER_STATUS}, TOFP_HOUR: {TOFP_HOUR}, TOFP_MIN: {TOFP_MIN}, TOFP_SEC: {TOFP_SEC}, TOFP_HUNDRETH: {TOFP_HUNDRETH}, LEAK_SENSOR: {LEAK_SENSOR}, SPEED_OF_SOUND: {SPEED_OF_SOUND}, TEMPERATURE: {TEMPERATURE}, CHECKSUM: {CHECKSUM}")