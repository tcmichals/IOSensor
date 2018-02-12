

#include <cstdint>
#include <array>

#include "startUp.h"


typedef 
    struct 
    {
        StartUpPriority Priority;
        const char *pName;
        startUpFunctor_t startUpFunc;
        bool isInit;
    }startFuncEntry_t;
        
        
static std::array<startFuncEntry_t, MAX_START_UP_FUNCS> gStartUp;
static uint32_t gNumEntries;

bool regStartUpFunction(StartUpPriority Priority,
                      const char *pName,
                     startUpFunctor_t func)
{
    
  if ( (gNumEntries +1) >=  gStartUp.size() )
  {
    return false;
  }
  
  if(func)
  {
    gStartUp[gNumEntries].Priority = Priority;
    gStartUp[gNumEntries].pName = pName;
    gStartUp[gNumEntries].startUpFunc = func;
    gStartUp[gNumEntries].isInit = true;
    
    if ( gStartUp[gNumEntries].startUpFunc != nullptr)
        gNumEntries++;
  }
  
  return true;
    
}

bool callStartupFunctions()
{
    bool rc = false;
    
    for( uint32_t _iter = static_cast<uint32_t>(StartUpPriority::Highest); 
        _iter < static_cast<uint32_t>(StartUpPriority::LastValue);
        ++_iter
       )
    {
        for(uint32_t _entry = 0; _entry < gNumEntries; ++_entry)
        {
            if ( (gStartUp[_entry].startUpFunc != nullptr) &&
                 (gStartUp[_entry].isInit) &&
                 (_iter == static_cast<uint32_t>(gStartUp[_entry].Priority))
                )
            {
                rc = gStartUp[_entry].startUpFunc();
                gStartUp[_entry].isInit= false;
            }
        }
    }
    
    return rc;
}

//eof
