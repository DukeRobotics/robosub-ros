# C++ plugin framework for V-REP

Compile with your C++ project.

Example plugin (uses also [v_repStubsGen](https://github.com/CoppeliaRobotics/v_repStubsGen)):

```
#include "v_repExtPluginSkeletonNG.h"
#include "v_repPlusPlus/Plugin.h"
#include "stubs.h"

void test(SScriptCallBack *p, const char *cmd, test_in *in, test_out *out)
{
    // ...
}

class Plugin : public vrep::Plugin
{
public:
    void onStart()
    {
        if(!registerScriptStuff())
            throw std::runtime_error("script stuff initialization failed");
    }
};

VREP_PLUGIN("PluginSkeletonNG", 1, Plugin)
```

See [v_repExtPluginSkeletonNG](https://github.com/CoppeliaRobotics/v_repExtPluginSkeletonNG) for a complete example.
