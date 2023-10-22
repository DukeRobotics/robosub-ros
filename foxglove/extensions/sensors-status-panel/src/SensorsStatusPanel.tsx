// TODO: (1) subscribe to all topics; (2) inside render context something update every single frame
import * as React from 'react';
import Table from '@mui/material/Table';
import TableBody from '@mui/material/TableBody';
import TableCell from '@mui/material/TableCell';
import TableContainer from '@mui/material/TableContainer';
import TableHead from '@mui/material/TableHead';
import TableRow from '@mui/material/TableRow';
import Paper from '@mui/material/Paper';

import { Immutable, PanelExtensionContext, RenderState, Topic, MessageEvent } from "@foxglove/studio";
import { useLayoutEffect, useEffect, useState, useRef, useMemo } from "react";
import * as ReactDOM from "react-dom";
import Alert from '@mui/material/Alert';

const topics_dict = {'DVL': '/sensors/dvl/odom', 
                    'IMU': '/vectornav/IMU', 
                    'Depth': '/sensors/depth', 
                    'DepthAI':'/camera/front/rgb/preview/compressed', 
                    'Mono': '/camera/usb_camera/compressed', 
                    'Sonar': '/sonar/status'}

//Reversed dictionary of topics_dict
const topics_dict_reversed: { [key: string]: string } = {};
for (const [key, value] of Object.entries(topics_dict)) {
  topics_dict_reversed[value] = key;
}

type SensorsTime = {} & Record<keyof typeof topics_dict, number>;
type ConnectStatus = {} & Record<keyof typeof topics_dict, boolean>;

// type SensorsTime = {
//   DVL: number;
//   IMU: number;
//   Depth: number;
//   DepthAI: number;
//   Mono: number;
//   Sonar: number;
// }

// type ConnectStatus = {
//   DVL: boolean;
//   IMU: boolean;
//   Depth: boolean;
//   DepthAI: boolean;
//   Mono: boolean;
//   Sonar: boolean;
// }

type State = {
  topic?: string;
  colorScheme?: RenderState["colorScheme"];
  sensorstime?: SensorsTime;
  connectStatus?: ConnectStatus;
  currentTime?: number;
};

function SensorsStatusPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [topics, setTopics] = useState<readonly Topic[] | undefined>();
  const [message, setMessage] = useState<any>();

  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  // Restore our state from the layout via the context.initialState property.
  const [state, setState] = useState<State>(() => {
    var Initialstate = context.initialState as State;
    Initialstate.sensorstime = {
      DVL: 0, 
      IMU: 0, 
      Depth: 0, 
      DepthAI: 0, 
      Mono: 0, 
      Sonar: 0
    }
    Initialstate.currentTime = 1
    Initialstate.connectStatus = {
      DVL: false, 
      IMU: false, 
      Depth: false, 
      DepthAI: false, 
      Mono: false, 
      Sonar: false
    }
    return Initialstate
  });

  // Get topics
  const imageTopics = useMemo(
    () => (topics ?? []),
    [topics],
  );

  useEffect(() => {
    // Save our state to the layout when the topic changes.
    context.saveState({ topic: state.topic });

    // if (state.topic) {
    //   // Subscribe to the new image topic when a new topic is chosen.
    //   context.subscribe([{ topic:"/sensors/dvl/odom" }, { topic: "/vectornav/IMU"}, { topic: "/sensors/depth"}, { topic: "/camera/front/rgb/preview/compressed"}, { topic: "/camera/usb_camera/compressed"}, { topic: "/sonar/status"}]);
    // }
    //make a list of all topics [{topic: topic1}, {topic: topic2 }]
    const topicList: { topic: string }[] = [];
    for (const [key, value] of Object.entries(topics_dict)) {
      topicList.push({topic: value})
    }
    //Subscribe to all topics
    context.subscribe(topicList)
  }, [context, state.topic]);

  // Choose our first available image topic as a default once we have a list of topics available.
  useEffect(() => {
    if (state.topic == undefined) {
      setState({ topic: imageTopics[0]?.name });
    }
  }, [state.topic, imageTopics]);

  // var lastRender = renderState.currentTime?.sec;
  // Setup our onRender function and start watching topics and currentFrame for messages.
  useLayoutEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setRenderDone(() => done);
      
      //Updates CurrentTime to the current time
      if (state.currentTime && state.currentTime >= 0)  {
        state.currentTime = renderState.currentTime?.sec
      }

      // if (renderState.currentFrame && renderState.currentFrame.length !== 0) {
      //   console.log(renderState.currentFrame[renderState.currentFrame.length - 1])
      // }

      //If sensorstime exists and the current frame exists (onRender was ran due to currentFrame changing)
      if (state.currentTime && state.sensorstime && state.connectStatus && renderState.currentFrame && renderState.currentFrame.length !== 0) {
        //Define the last frame 
        const lastFrame = renderState.currentFrame[renderState.currentFrame.length - 1] as MessageEvent<any>;
  
        //Switch statements on the topic of the last frame 
        
        if(lastFrame){
          // switch(lastFrame.topic) {
          //   case "/sensors/dvl/odom":
          //     state.sensorstime.DVL = state.currentTime;
          //     state.connectStatus.DVL = true;
          //     break;
          //   case "/vectornav/IMU":
          //     state.sensorstime.IMU = state.currentTime;
          //     state.connectStatus.IMU = true;
          //     break;
          //   case "/sensors/depth":
          //     state.sensorstime.Depth = state.currentTime;
          //     state.connectStatus.Depth = true;
          //     break;
          //   case "/camera/front/rgb/preview/compressed":
          //     state.sensorstime.DepthAI = state.currentTime;
          //     state.connectStatus.DepthAI = true;
          //     break;
          //   case "/camera/usb_camera/compressed":
          //     state.sensorstime.Mono = state.currentTime;
          //     state.connectStatus.Mono = true;
          //     break;
          //   case "/sonar/status":
          //     state.sensorstime.Sonar = state.currentTime;
          //     state.connectStatus.Sonar = true;
          //     break;
          // }
          // try catch statement to see if the topic is in the dictionary
          //force lastFrame.topic to not be undefined

          try {
            //force sensorName to not be undefined
            const sensorName = topics_dict_reversed[lastFrame.topic!] as keyof typeof topics_dict;
            state.sensorstime[sensorName] = state.currentTime;
            state.connectStatus[sensorName!] = true;
          } catch (error) {
            console.log(error);
          }
          
        }
      
    } 

    if (state.connectStatus && state.sensorstime && state.currentTime){
      //Compare current time to each sensorstime attribute
      // if (state.currentTime - state.sensorstime.DVL > 5)  {state.connectStatus.DVL = false}
      // if (state.currentTime - state.sensorstime.IMU > 5)  {state.connectStatus.IMU = false}
      // if (state.currentTime - state.sensorstime.Depth > 5)  {state.connectStatus.Depth = false}  
      // if (state.currentTime - state.sensorstime.DepthAI > 5)  {state.connectStatus.DepthAI = false}  
      // if (state.currentTime - state.sensorstime.Mono > 5)  {state.connectStatus.Mono = false}  
      // if (state.currentTime - state.sensorstime.Sonar > 5)  {state.connectStatus.Sonar = false}  
      for (const [key, value] of Object.entries(topics_dict)) {
        if (state.currentTime - state.sensorstime[key as keyof typeof topics_dict] > 5) {
          state.connectStatus[key as keyof typeof topics_dict] = false;
        }
      }
    }

      setTopics(renderState.topics);
      setState((oldState) => ({ ...oldState, colorScheme: renderState.colorScheme }));
      
      // Save the most recent message on our topic.
      if (renderState.currentFrame && renderState.currentFrame.length > 0) {
        setMessage(renderState.currentFrame[renderState.currentFrame.length - 1]);
      }
      
    };
    context.watch("currentTime");
    context.watch("topics");
    context.watch("currentFrame");
    context.watch("colorScheme");

  }, [context]);

  // Call our done function at the end of each render.
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

/*
Might not need asynch and table?

(async function checkTimeSinceRender() {
    const waitInSeconds = 1;
    var beginning = Date.now() - 0 //TODO: state.sensorstime.SENSO;
    switch(state.topic) {
      case "/sensors/dvl/odom":
        timeDiff = renderState.currentTime?.sec - state.sensorstime.DVL;
        if (timeDiff > )
        break;
      case "2nd-topicname":
        state.sensorstime.IMU =;
        break;
      case "/sensors/depth":
        state.sensorstime.Depth = ;
        break;
      case "/camera/front/rgb/preview/compressed":
        state.sensorstime.DepthAI = ;
        break;
      case "/camera/usb_camera/compressed":
        state.sensorstime.Mono = ;
        break;
      case "/sonar/status":
        state.sensorstime.Sonar = ;
        break;
    }
    await sleep(waitInSeconds * 1000);
    state.topic 
    
})();
//TODO: Make if statement to see if sensorstime exists
const SensorTable = [
  createData('DVL', '/sensors/dvl/odom', ( state.sensorstime.DVL ? true : false)),
  createData('IMU','/vectornav/IMU',true),
  createData('Depth', '/sensors/depth',true),
  createData('DepthAI Camera', '/camera/front/rgb/preview/compressed', true),
  createData('Mono Camera','/camera/usb_camera/compressed',true),
  createData('Sonar', '/sonar/status', true),
];
*/

//create a table of all the sensors and their status with the goal of being put into a Table component using a for loop


  return (
    <div style={{ height: "100%", padding: "1rem" }}>
      <h2>Sensors Status Panel</h2>
      {context.subscribe == undefined && (
        <Alert variant="filled" severity="error">Subscribing to topics is not supported by this connection</Alert>
      )}
      <div>
        <label>Choose a topic to display: </label>
        <select
          value={state.topic}
          onChange={(event) => setState({ topic: event.target.value })}
          style={{ flex: 1 }}
        >
          {imageTopics.map((topic) => (
            <option key={topic.name} value={topic.name}>
              {topic.name}
            </option>
          ))}
        </select>

      <TableContainer component={Paper}>
        <Table sx={{ minWidth: 650 }} aria-label="simple table">
          <TableHead>
            <TableRow>
              <TableCell>Sensor Name</TableCell>
              <TableCell align="right">Topic Name</TableCell>
              <TableCell align="right">Sensor Publishing?</TableCell>
            </TableRow>
          </TableHead>
          <TableBody>

            {Object.entries(topics_dict).map(([sensor, topic]) => (
              <TableRow key={sensor} sx={{ '&:last-child td, &:last-child th': { border: 0 } }}>
                <TableCell>{sensor}</TableCell>
                <TableCell align="right">{topic}</TableCell>
                <TableCell align="right">
                  {state.connectStatus?.[sensor as keyof typeof topics_dict] ? "Connected" : "Disconnected"}
                </TableCell>
              </TableRow>
          ))}
        </TableBody>
        </Table>
      </TableContainer>

      </div>
    </div>
  );
}

function createData(
  Sensor_Name: string,
  Topic_Path: string,
  Sensor_Publishing: boolean,
) {
  return { Sensor_Name, Topic_Path, Sensor_Publishing };
}


export function initSensorsStatusPanel(context: PanelExtensionContext): () => void {
  ReactDOM.render(<SensorsStatusPanel context={context} />, context.panelElement);

  // Return a function to run when the panel is removed
  return () => {
    ReactDOM.unmountComponentAtNode(context.panelElement);
  };
}