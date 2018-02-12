
#pragma once

#ifdef __cplusplus
#include <cstdint>
//Start up Priority

#define MAX_START_UP_FUNCS 20

extern "C"
{
    
enum class StartUpPriority: uint32_t
{
    None,
    Highest,
    _2,
    _3,
    _4,
    _5,
    _6,
    _7,
    _8,
    _9,
    _10,
    _11,
    _12,
    _13,
    _14,
    _15,
    Last,
    LastValue,
};    
typedef bool (*startUpFunctor_t )(void);
bool regStartUpFunction(StartUpPriority Priority,
                      	const char *pName,
                     	startUpFunctor_t func);    
#else
#include <stdbool.h>
#endif


bool callStartupFunctions(void);

#ifdef __cplusplus
}
#endif

//eof






