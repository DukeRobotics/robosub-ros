#if !defined(V_REPTYPES_INCLUDED_)
#define V_REPTYPES_INCLUDED_

// Various types used in the interface functions:
typedef unsigned char simBool;
typedef char simChar;
typedef int simInt;
typedef float simFloat;
typedef double simDouble;
typedef void simVoid;
typedef unsigned char simUChar;
typedef unsigned int simUInt;
typedef unsigned long long int simUInt64;

struct SScriptCallBack
{
    simInt objectID;
    simInt scriptID;
    simInt stackID;
    simChar waitUntilZero;
    simChar* raiseErrorWithMessage;
};

struct SLuaCallBack
{
    simInt objectID;
    simBool* inputBool;
    simInt* inputInt;
    simFloat* inputFloat;
    simChar* inputChar;
    simInt inputArgCount;
    simInt* inputArgTypeAndSize;
    simBool* outputBool;
    simInt* outputInt;
    simFloat* outputFloat;
    simChar* outputChar;
    simInt outputArgCount;
    simInt* outputArgTypeAndSize;
    simChar waitUntilZero;
    simChar* inputCharBuff;
    simChar* outputCharBuff;
    simInt scriptID;
    simDouble* inputDouble;
    simDouble* outputDouble;
};

typedef int (*contactCallback)(int,int,int,int*,float*);
typedef int (*jointCtrlCallback)(int,int,int,const int*,const float*,float*);

#endif // !defined(V_REPTYPES_INCLUDED_)
