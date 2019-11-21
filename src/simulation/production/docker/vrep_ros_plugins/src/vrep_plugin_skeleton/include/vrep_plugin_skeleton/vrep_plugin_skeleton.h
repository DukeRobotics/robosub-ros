#ifndef V_REPEXTROSSKELETON_H
#define V_REPEXTROSSKELETON_H

#define VREP_DLLEXPORT extern "C"

// The 3 required entry points of the V-REP plugin:
VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt);
VREP_DLLEXPORT void v_repEnd();
VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData);

#endif
