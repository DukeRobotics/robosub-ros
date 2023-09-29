import time

line = "7D002D00B300800080008000800000000000000000FF0080008000800080000000000F013A381F0000F40534083208"

#calculate delta time

print(int("FF",16))

curTime = time.time()

byte_string = bytearray.fromhex(line)

bytes = [line[i:i+2] for i in range(0, len(line), 2)]

DATA_ID = int(bytes[0], 16)
DATA_STRUCT = int(bytes[1], 16)
NUM_BYTES = int(bytes[3] + bytes[2], 16)
SYS_CONFIG = int(bytes[4], 16)
X_VEL = int(bytes[6] + bytes[5], 16)
Y_VEL = int(bytes[8] + bytes[7], 16)
Z_VEL = int(bytes[10] + bytes[9], 16)
E_VEL = int(bytes[12] + bytes[11], 16)
BM1 = int(bytes[14] + bytes[13], 16)
BM2 = int(bytes[16] + bytes[15], 16)
BM3 = int(bytes[18] + bytes[17], 16)
BM4 = int(bytes[20] + bytes[19], 16)
BOTTOM_STATUS = int(bytes[21], 16)
VEL_1 = int(bytes[23] + bytes[22], 16)
VEL_2 = int(bytes[25] + bytes[24], 16)
VEL_3 = int(bytes[27] + bytes[26], 16)
VEL_4 = int(bytes[29] + bytes[28], 16)
REF_LAYER_START = int(bytes[31] + bytes[30], 16)
REF_LAYER_END = int(bytes[33] + bytes[32], 16)
REF_LAYER_STATUS = int(bytes[34], 16)
TOFP_HOUR = int(bytes[35], 16)
TOFP_MIN = int(bytes[36], 16)
TOFP_SEC = int(bytes[37], 16)
TOFP_HUNDRETH = int(bytes[38], 16)
LEAK_SENSOR = int(bytes[40] + bytes[39], 16)
SPEED_OF_SOUND = int(bytes[42] + bytes[41], 16)
TEMPERATURE = int(bytes[44] + bytes[43], 16)
CHECKSUM = int(bytes[46] + bytes[45], 16)

print(f"delta time: {time.time() - curTime}")

print(f"DATA_ID: {DATA_ID}, DATA_STRUCT: {DATA_STRUCT}, NUM_BYTES: {NUM_BYTES}, SYS_CONFIG: {SYS_CONFIG}, X_VEL: {X_VEL}, Y_VEL: {Y_VEL}, Z_VEL: {Z_VEL}, E_VEL: {E_VEL}, BM1: {BM1}, BM2: {BM2}, BM3: {BM3}, BM4: {BM4}, BOTTOM_STATUS: {BOTTOM_STATUS}, VEL_1: {VEL_1}, VEL_2: {VEL_2}, VEL_3: {VEL_3}, VEL_4: {VEL_4}, REF_LAYER_START: {REF_LAYER_START}, REF_LAYER_END: {REF_LAYER_END}, REF_LAYER_STATUS: {REF_LAYER_STATUS}, TOFP_HOUR: {TOFP_HOUR}, TOFP_MIN: {TOFP_MIN}, TOFP_SEC: {TOFP_SEC}, TOFP_HUNDRETH: {TOFP_HUNDRETH}, LEAK_SENSOR: {LEAK_SENSOR}, SPEED_OF_SOUND: {SPEED_OF_SOUND}, TEMPERATURE: {TEMPERATURE}, CHECKSUM: {CHECKSUM}")