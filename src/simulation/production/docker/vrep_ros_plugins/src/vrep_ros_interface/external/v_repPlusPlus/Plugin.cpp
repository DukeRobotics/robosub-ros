#include "Plugin.h"

#include <string>
#include <vector>
#include <cstring>

namespace vrep
{
    void Plugin::onStart()
    {
    }

    void Plugin::onEnd()
    {
    }

    void * Plugin::onMessage(int message, int *auxiliaryData, void *customData, int *replyData)
    {
        int errorModeSaved;
        simGetIntegerParameter(sim_intparam_error_report_mode, &errorModeSaved);
        simSetIntegerParameter(sim_intparam_error_report_mode, sim_api_errormessage_ignore);
        void *retVal = NULL;

        switch(message)
        {
        case sim_message_eventcallback_instancepass:
            /*
            Called once every main client application loop pass.
            auxiliaryData[0] contains event flags of events that happened since last time.
            If you react to some of below event flags, make sure you do not react to their
            equivalent event callback message (e.g. sim_message_eventcallback_sceneloaded is
            similar to below's bit3 set)
            */
            {
                InstancePassFlags flags;
                flags.objectsErased         = (auxiliaryData[0] & (1 <<  0)) > 0;
                flags.objectsCreated        = (auxiliaryData[0] & (1 <<  1)) > 0;
                flags.modelLoaded           = (auxiliaryData[0] & (1 <<  2)) > 0;
                flags.sceneLoaded           = (auxiliaryData[0] & (1 <<  3)) > 0;
                flags.undoCalled            = (auxiliaryData[0] & (1 <<  4)) > 0;
                flags.redoCalled            = (auxiliaryData[0] & (1 <<  5)) > 0;
                flags.sceneSwitched         = (auxiliaryData[0] & (1 <<  6)) > 0;
                flags.editModeActive        = (auxiliaryData[0] & (1 <<  7)) > 0;
                flags.objectsScaled         = (auxiliaryData[0] & (1 <<  8)) > 0;
                flags.selectionStateChanged = (auxiliaryData[0] & (1 <<  9)) > 0;
                flags.keyPressed            = (auxiliaryData[0] & (1 << 10)) > 0;
                flags.simulationStarted     = (auxiliaryData[0] & (1 << 11)) > 0;
                flags.simulationEnded       = (auxiliaryData[0] & (1 << 12)) > 0;
                flags.scriptCreated         = (auxiliaryData[0] & (1 << 13)) > 0;
                flags.scriptErased          = (auxiliaryData[0] & (1 << 14)) > 0;

                onInstancePass(flags, firstInstancePass); // for backward compatibility

                if(firstInstancePass) onFirstInstancePass(flags);
                else onInstancePass(flags);

                firstInstancePass = false;
            }
            break;
#if VREP_PROGRAM_FULL_VERSION_NB >= 3060104 // 3.6.1.rev4
        case sim_message_eventcallback_lastinstancepass:
            /*
            called on the last client application loop pass (the instancepass message is not sent)
            */
            {
                onLastInstancePass();
            }
            break;
#endif // VREP_PROGRAM_FULL_VERSION_NB >= 3060104
        case sim_message_eventcallback_instanceswitch:
            /*
            scene was switched (react to this message in a similar way as you would react to
            a full scene content change)

            auxiliaryData[0]: do not use
            auxiliaryData[1]=current scene unique ID
            */
            {
                onInstanceSwitch(auxiliaryData[1]);
            }
            break;
        case sim_message_eventcallback_instanceabouttoswitch:
            /*
            we are about to switch to a different scene

            auxiliaryData[0]: do not use
            auxiliaryData[1]=next scene unique ID
            */
            {
                onInstanceAboutToSwitch(auxiliaryData[1]);
            }
            break;
        case sim_message_eventcallback_menuitemselected:
            /*
            (called from the UI thread)
            auxiliaryData[0]=handle of the item
            auxiliaryData[1]=state of the item
            */
            {
                onMenuItemSelected(auxiliaryData[0], auxiliaryData[1]);
            }
            break;
        case sim_message_eventcallback_broadcast:
            /*
            called when simBroadcastMessage is called

            auxiliaryData[0]=header (e.g. a large, random identifier, or your V-REP serial number)
            auxiliaryData[1]=message ID
            */
            {
                onBroadcast(auxiliaryData[0], auxiliaryData[1]);
            }
            break;
        case sim_message_eventcallback_scenesave:
            /*
            about to save a scene
            */
            {
                onSceneSave();
            }
            break;
        case sim_message_eventcallback_modelsave:
            /*
            about to save a model (current selection will be saved)
            */
            {
                onModelSave();
            }
            break;
        case sim_message_eventcallback_moduleopen:
            /*
            called when simOpenModule in Lua is called
            customData=name of the plugin to execute the command, or NULL if all should
                        execute the command
            */
            {
                onModuleOpen((char *)customData);
            }
            break;
        case sim_message_eventcallback_modulehandle:
            /*
            called when simHandleModule in Lua is called with second argument false
            customData=name of the plugin to execute the command, or NULL if all should
                    execute the command
            */
            {
                onModuleHandle((char *)customData);
            }
            break;
        case sim_message_eventcallback_modulehandleinsensingpart:
            /*
            called when simHandleModule in Lua is called with second arg. true
            customData=name of the plugin to execute the command, or NULL if all should
                    execute the command
            */
            {
                onModuleHandleInSensingPart((char *)customData);
            }
            break;
        case sim_message_eventcallback_moduleclose:
            /*
            called when simCloseModule in Lua is called
            customData=name of the plugin to execute the command, or NULL if all should
                    execute the command
            */
            {
                onModuleClose((char *)customData);
            }
            break;
        case sim_message_eventcallback_renderingpass:
            /*
            (called from the UI thread)
            called just before the scene is rendered. Enable via simEnableEventCallback.
            */
            {
                onRenderingPass();
            }
            break;
        case sim_message_eventcallback_beforerendering:
            /*
            called just before the scene is rendered, but still from the main simulation
                    thread. Only called when in non-threaded rendering mode.
            */
            {
                onBeforeRendering();
            }
            break;
        case sim_message_eventcallback_imagefilter_enumreset:
            /*
            "reset enumeration" message for filter plugins
            */
            {
                onImageFilterEnumReset();
            }
            break;
        case sim_message_eventcallback_imagefilter_enumerate:
            /*
            "enumerate" message for filter plugins

            replyData[0]=header ID
            replyData[1]=filterID. A positive ID (including 0) represents a filter whose
                    parameters can be edited. A filter with a negative ID cannot be edited.

            Return the filter's name as the callback return value (allocate the buffer with
            the simCreateBuffer function)
            */
            {
                std::string name;
                onImageFilterEnumerate(replyData[0], replyData[1], name);
                if(name.length())
                {
                    simChar *ret = simCreateBuffer((simInt)name.length() + 1);
                    std::strncpy(ret, name.c_str(), name.length());
                    retVal = ret;
                }
            }
            break;
        case sim_message_eventcallback_imagefilter_adjustparams:
            /*
            (called from the UI thread)
            "edit filter parameters" message for filter plugins

            auxiliaryData[0]=header ID
            auxiliaryData[1]=filter ID
            auxiliaryData[2]=size in bytes of the filter parameter buffer to be edited
            customData=filter parameter buffer. Can be NULL (is NULL when filter parameters
                    were never edited)
            replyData[0]=size in bytes of the edited filter parameter buffer returned as the
                    callback return buffer. That buffer has to be allocated with the
                    simCreateBuffer command and is automatically released by V-REP upon
                    callback return.
            */
            {
                onImageFilterAdjustParams(auxiliaryData[0], auxiliaryData[1], auxiliaryData[2], customData, replyData[0], retVal);
            }
            break;
        case sim_message_eventcallback_imagefilter_process:
            /*
            (called from the UI thread)
            "do image processing" message for filter plugins

            auxiliaryData[0]=header ID
            auxiliaryData[1]=filter ID
            auxiliaryData[2]=resolution X
            auxiliaryData[3]=resolution Y
            auxiliaryData[4]=vision sensor handle (available from V-REP 3.1.0 only)
            customData[0]=input image (size: x*y*3 floats)
            customData[1]=input depth image (size: x*y floats)
            customData[2]=work image (size: x*y*3 floats)
            customData[3]=buffer image 1 (size: x*y*3 floats)
            customData[4]=buffer image 2 (size: x*y*3 floats)
            customData[5]=output image (size: x*y*3 floats)
            customData[6]=filter parameter buffer (size: custom), can be NULL (is NULL when
                        filter parameters were never edited)
            replyData[0]=1 if the sensor should trigger a detection, 0 otherwise
            replyData[1]=number of float values returned, representing auxiliary information
                        (that can be retrieved with simHandleVisionSensor). The auxiliary
                        information can represent a vector, a direction, any result from an
                        image processing algorithm. Allocate the return float buffer with
                        simCreateBuffer
            */
            {
                simFloat **b = (simFloat**)customData;
                std::vector<simFloat> r = onImageFilterProcess(auxiliaryData[0], auxiliaryData[1], auxiliaryData[2], auxiliaryData[3], auxiliaryData[4], b[0], b[1], b[2], b[3], b[4], b[5], (void*)b[6], replyData[0]);
                if(r.size())
                {
                    simFloat *ret2 = (simFloat*)simCreateBuffer(sizeof(simFloat) * (simInt)r.size());
                    for(size_t i = 0; i < r.size(); i++) ret2[i] = r[i];
                    replyData[1] = (simInt)r.size();
                    retVal = ret2;
                }
            }
            break;
        case sim_message_eventcallback_abouttoundo:
            /*
            the undo button was hit and a previous state is about to be restored
            */
            {
                onAboutToUndo();
            }
            break;
        case sim_message_eventcallback_undoperformed:
            /*
            the undo button was hit and a previous state was restored
            */
            {
                onUndo();
            }
            break;
        case sim_message_eventcallback_abouttoredo:
            /*
            the redo button was hit and a future state is about to be restored
            */
            {
                onAboutToRedo();
            }
            break;
        case sim_message_eventcallback_redoperformed:
            /*
            the redo button was hit and a future state was restored
            */
            {
                onRedo();
            }
            break;
        case sim_message_eventcallback_scripticondblclick:
            /*
            (called from the UI thread)
            a script icon in the hierarchy view was double-clicked

            auxiliaryData[0]=object handle of the object associated with the script
            replyData[0]: set to 1 if you do not want the double-click action to open the
                        script editor
            */
            {
                onScriptIconDblClick(auxiliaryData[0], replyData[0]);
            }
            break;
        case sim_message_eventcallback_simulationabouttostart:
            /*
            simulation will start
            */
            {
                onSimulationAboutToStart();
            }
            break;
        case sim_message_eventcallback_simulationabouttoend:
            /*
            simulation will end
            */
            {
                onSimulationAboutToEnd();
            }
            break;
        case sim_message_eventcallback_simulationended:
            /*
            simulation has ended
            */
            {
                onSimulationEnded();
            }
            break;
        case sim_message_eventcallback_keypress:
            /*
            (called from the UI thread)
            auxiliaryData[0]=key, auxiliaryData[1]=ctrl and shift key state
            */
            {
                onKeyPress(auxiliaryData[0], auxiliaryData[1]);
            }
            break;
        case sim_message_eventcallback_bannerclicked:
            /*
            (called from the UI thread)
            called when a banner was clicked (auxiliaryData[0]=banner ID)
            */
            {
                onBannerClicked(auxiliaryData[0]);
            }
            break;
        case sim_message_eventcallback_refreshdialogs:
            /*
            (called from the UI thread)
            called just before disloags are refreshed in V-REP.

            auxiliaryData[0]=refresh degree (0=light, 1=medium, 2=full)
            */
            {
                onRefreshDialogs(auxiliaryData[0]);
            }
            break;
        case sim_message_eventcallback_sceneloaded:
            /*
            called after a scene was loaded
            */
            {
                onSceneLoaded();
            }
            break;
        case sim_message_eventcallback_modelloaded:
            /*
            called after a model was loaded
            */
            {
                onModelLoaded();
            }
            break;
        case sim_message_eventcallback_guipass:
            /*
            (called from the UI thread)
            Called on a regular basis from the GUI thread.
            */
            {
                onGuiPass();
            }
            break;
        case sim_message_eventcallback_mainscriptabouttobecalled:
            /*
            Called just before the main script is called. If a plugin intercepts this
            message and writes a value different from -1 into replyData[0], the main script
            will not be called.
            */
            {
                onMainScriptAboutToBeCalled(replyData[0]);
            }
            break;
        case sim_message_eventcallback_rmlpos:
            /*
            the command simRMLPos was called. The appropriate plugin should handle the call
            */
            {
                onRMLPos();
            }
            break;
        case sim_message_eventcallback_rmlvel:
            /*
            the command simRMLVel was called. The appropriate plugin should handle the call
            */
            {
                onRMLVel();
            }
            break;
        case sim_message_eventcallback_rmlstep:
            /*
            the command simRMLStep was called. The appropriate plugin should handle the call
            */
            {
                onRMLStep();
            }
            break;
        case sim_message_eventcallback_rmlremove:
            /*
            the command simRMLRemove was called. The appropriate plugin should handle the call
            */
            {
                onRMLRemove();
            }
            break;
        case sim_message_eventcallback_pathplanningplugin:
            /*
            a path planning function was called. The appropriate plugin (i.e. 'PathPlanning')
            should handle the call
            */
            {
                onPathPlanningPlugin();
            }
            break;
        case sim_message_eventcallback_colladaplugin:
            /*
            a collada plugin function was called. The appropriate plugin (i.e. 'Collada')
            should handle the call
            */
            {
                onColladaPlugin();
            }
            break;
        case sim_message_eventcallback_opengl:
            /*
            (called from the UI thread)
            (Enable via simEnableEventCallback)
            the user can perform openGl calls from the plugin, in order to draw custom
            graphics into the V-REP scene. This has a different functionality from the
            simAddDrawingObject API function. Following data is sent to the plugins:

            auxiliaryData[0]=index of the program location where the call occured
                        (e.g. 0=before first V-REP rendering, 5=after last V-REP rendering).
                        Refer to the source code for details.
            auxiliaryData[1]=current rendering attributes
            auxiliaryData[2]=handle of the camera
            auxiliaryData[3]=index of the view, or -1 if view is unassociated
            */
            {
                onOpenGL(auxiliaryData[0], auxiliaryData[1], auxiliaryData[2], auxiliaryData[3]);
            }
            break;
        case sim_message_eventcallback_openglframe:
            /*
            (called from the UI thread)
            (Enable via simEnableEventCallback)
            a callback with the full rendered opengl frame data (that can be modified, then returned):

            customData (unsigned char*): RGB data of the image.
            auxiliaryData[0]=picture size X
            auxiliaryData[1]=picture size Y
            auxiliaryData[2]=0
            auxiliaryData[3]=0. If you want V-REP to take into account any modification
                        on the buffer, write 1 in here.
            */
            {
                onOpenGLFrame(auxiliaryData[0], auxiliaryData[1], auxiliaryData[3]);
            }
            break;
        case sim_message_eventcallback_openglcameraview:
            /*
            (called from the UI thread)
            (Enable via simEnableEventCallback)
            a callback with the rendered opengl camera view (that can be modified, then returned):

            customData (unsigned char*): RGB data of the image.
            auxiliaryData[0]=picture size X
            auxiliaryData[1]=picture size Y
            auxiliaryData[2]=view index
            auxiliaryData[3]=0. If you want V-REP to take into account any modification
                        on the buffer, write 1 in here.
            */
            {
                onOpenGLCameraView(auxiliaryData[0], auxiliaryData[1], auxiliaryData[2], auxiliaryData[3]);
            }
            break;
        case sim_message_eventcallback_proxsensorselectdown:
            /*
            a "geometric" click select (mouse down) was registered. Not generated if the
            ctrl or shift key is down. A geometric click is generated in a non-delayed manner.
            See also sim_message_eventcallback_pickselectdown hereafter. Enable with
            sim_intparam_prox_sensor_select_down.

            auxiliaryData[0]=objectID
            customData[0]-customData[2]=coordinates of clicked point
            customData[3]-customData[5]=normal vector of clicked surface
            */
            {
                simFloat *f = (simFloat*)customData;
                onProxSensorSelectDown(auxiliaryData[0], f, f+3);
            }
            break;
        case sim_message_eventcallback_proxsensorselectup:
            /*
            a "geometric" click select (mouse up) was registered. Not generated if the ctrl
            or shift key is down. A geometric click is generated in a non-delayed manner.
            Enable with sim_intparam_prox_sensor_select_up.

            auxiliaryData[0]=objectID
            customData[0]-customData[2]=coordinates of clicked point
            customData[3]-customData[5]=normal vector of clicked surface
            */
            {
                simFloat *f = (simFloat*)customData;
                onProxSensorSelectUp(auxiliaryData[0], f, f+3);
            }
            break;
        case sim_message_eventcallback_pickselectdown:
            /*
            (called from the UI thread)
            a "pick" click select (mouse down) was registered. Not generated if the ctrl or
            shift key is down. A pick click is generated in a delayed manner. See also
            sim_message_eventcallback_proxsensorselectdown here above.

            auxiliaryData[0]=objectID or base object ID (if part of a model and select model
                        base instead is checked)
            */
            {
                onPickSelectDown(auxiliaryData[0]);
            }
            break;
        }

        // Keep following unchanged:
        simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved);
        return retVal;
    }

    LIBRARY Plugin::loadVrepLibrary()
    {
        LIBRARY lib = NULL;
        char curDirAndFile[1024];
#ifdef _WIN32
#ifdef QT_COMPIL
        _getcwd(curDirAndFile, sizeof(curDirAndFile));
#else
        ::GetModuleFileNameA(NULL, curDirAndFile, 1023);
        ::PathRemoveFileSpecA(curDirAndFile);
#endif
#elif defined (__linux) || defined (__APPLE__)
        getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif
        std::string currentDirAndPath(curDirAndFile);
        std::string temp(currentDirAndPath);
#ifdef _WIN32
        temp += "\\v_rep.dll";
#elif defined (__linux)
        temp += "/libv_rep.so";
#elif defined (__APPLE__)
        temp += "/libv_rep.dylib";
#endif
        lib = ::loadVrepLibrary(temp.c_str());
        if(lib == NULL)
        {
            throw std::runtime_error("could not find or correctly load the V-REP library");
        }
        if(::getVrepProcAddresses(lib)==0)
        {
            ::unloadVrepLibrary(lib);
            throw std::runtime_error("could not find all required functions in the V-REP library");
        }
        return lib;
    }

    void Plugin::onInstancePass(const InstancePassFlags &flags, bool first)
    {
    }

    void Plugin::onInstancePass(const InstancePassFlags &flags)
    {
    }

    void Plugin::onFirstInstancePass(const InstancePassFlags &flags)
    {
    }

    void Plugin::onLastInstancePass()
    {
    }

    void Plugin::onInstanceSwitch(int sceneID)
    {
    }

    void Plugin::onInstanceAboutToSwitch(int sceneID)
    {
    }

    void Plugin::onMenuItemSelected(int itemHandle, int itemState)
    {
    }

    void Plugin::onBroadcast(int header, int messageID)
    {
    }

    void Plugin::onSceneSave()
    {
    }

    void Plugin::onModelSave()
    {
    }

    void Plugin::onModuleOpen(char *name)
    {
    }

    void Plugin::onModuleHandle(char *name)
    {
    }

    void Plugin::onModuleHandleInSensingPart(char *name)
    {
    }

    void Plugin::onModuleClose(char *name)
    {
    }

    void Plugin::onRenderingPass()
    {
    }

    void Plugin::onBeforeRendering()
    {
    }

    void Plugin::onImageFilterEnumReset()
    {
    }

    void Plugin::onImageFilterEnumerate(int &headerID, int &filterID, std::string &name)
    {
    }

    void Plugin::onImageFilterAdjustParams(int headerID, int filterID, int bufferSize, void *buffer, int &editedBufferSize, void *&editedBuffer)
    {
    }

    std::vector<simFloat> Plugin::onImageFilterProcess(int headerID, int filterID, int resX, int resY, int visionSensorHandle, simFloat *inputImage, simFloat *depthImage, simFloat *workImage, simFloat *bufferImage1, simFloat *bufferImage2, simFloat *outputImage, void *filterParamBuffer, int &triggerDetectionn)
    {
        return std::vector<simFloat>();
    }

    void Plugin::onAboutToUndo()
    {
    }

    void Plugin::onUndo()
    {
    }

    void Plugin::onAboutToRedo()
    {
    }

    void Plugin::onRedo()
    {
    }

    void Plugin::onScriptIconDblClick(int objectHandle, int &dontOpenEditor)
    {
    }

    void Plugin::onSimulationAboutToStart()
    {
    }

    void Plugin::onSimulationAboutToEnd()
    {
    }

    void Plugin::onSimulationEnded()
    {
    }

    void Plugin::onKeyPress(int key, int mods)
    {
    }

    void Plugin::onBannerClicked(int bannerID)
    {
    }

    void Plugin::onRefreshDialogs(int refreshDegree)
    {
    }

    void Plugin::onSceneLoaded()
    {
    }

    void Plugin::onModelLoaded()
    {
    }

    void Plugin::onGuiPass()
    {
    }

    void Plugin::onMainScriptAboutToBeCalled(int &out)
    {
    }

    void Plugin::onRMLPos()
    {
    }

    void Plugin::onRMLVel()
    {
    }

    void Plugin::onRMLStep()
    {
    }

    void Plugin::onRMLRemove()
    {
    }

    void Plugin::onPathPlanningPlugin()
    {
    }

    void Plugin::onColladaPlugin()
    {
    }

    void Plugin::onOpenGL(int programIndex, int renderingAttributes, int cameraHandle, int viewIndex)
    {
    }

    void Plugin::onOpenGLFrame(int sizeX, int sizeY, int &out)
    {
    }

    void Plugin::onOpenGLCameraView(int sizeX, int sizeY, int viewIndex, int &out)
    {
    }

    void Plugin::onProxSensorSelectDown(int objectID, simFloat *clickedPoint, simFloat *normalVector)
    {
    }

    void Plugin::onProxSensorSelectUp(int objectID, simFloat *clickedPoint, simFloat *normalVector)
    {
    }

    void Plugin::onPickSelectDown(int objectID)
    {
    }
}
