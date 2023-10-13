// TODO: (1) subscribe to all topics; (2) inside render context something update every single frame
import * as React from 'react';
import Table from '@mui/material/Table';
import TableBody from '@mui/material/TableBody';
import TableCell from '@mui/material/TableCell';
import TableContainer from '@mui/material/TableContainer';
import TableHead from '@mui/material/TableHead';
import TableRow from '@mui/material/TableRow';
import Paper from '@mui/material/Paper';

import { PanelExtensionContext, RenderState, Topic, MessageEvent } from "@foxglove/studio";
import { useLayoutEffect, useEffect, useState, useRef, useMemo } from "react";
import ReactDOM from "react-dom";
import ReactJson from "react-json-view";
import Alert from '@mui/material/Alert';

type SensorsTime = {
  DVL: number;
  IMU: number;
  Depth: number;
  DepthAI: number;
  Mono: number;
  Sonar: number;
}

type State = {
  topic?: string;
  colorScheme?: RenderState["colorScheme"];
  sensorstime?: SensorsTime;
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

    if (state.topic) {
      // Subscribe to the new image topic when a new topic is chosen.
      context.subscribe([{ topic: state.topic }]);
    }
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
    context.onRender = (renderState: RenderState, done) => {
      setRenderDone(() => done);

      //Updates CurrentTime to the current time
      if (state.currentTime)  {
        state.currentTime = renderState.currentTime?.sec
      }
      //If sensorstime exists and the current frame exists (onRender was ran due to currentFrame changing)
      if (state.currentTime && state.sensorstime && renderState.currentFrame && renderState.currentFrame.length > 0) {
        
        //Define the last frame 
        const lastFrame = renderState.currentFrame[-1] as MessageEvent<any>;

        //Switch statements on the topic of the last frame 
        switch(lastFrame.topic) {
          case "/sensors/dvl/odom":
            state.sensorstime.DVL = state.currentTime;
            break;
          case "/vectornav/IMU)":
            state.sensorstime.IMU = state.currentTime;
            break;
          case "/sensors/depth":
            state.sensorstime.Depth = state.currentTime;
            break;
          case "/camera/front/rgb/preview/compressed":
            state.sensorstime.DepthAI = state.currentTime;
            break;
          case "/camera/usb_camera/compressed":
            state.sensorstime.Mono = state.currentTime;
            break;
          case "/sonar/status":
            state.sensorstime.Sonar = state.currentTime;
            break;
        }
      
      //Compare current time to each sensorstime attribute
      if (state.currentTime - state.sensorstime.DVL > 5)  {/*Ahh scremaing we have an inactive sensor*/}
      if (state.currentTime - state.sensorstime.IMU > 5)  {/*Ahh scremaing we have an inactive sensor*/}
      if (state.currentTime - state.sensorstime.Depth > 5)  {/*Ahh scremaing we have an inactive sensor*/}  
      if (state.currentTime - state.sensorstime.DepthAI > 5)  {/*Ahh scremaing we have an inactive sensor*/}  
      if (state.currentTime - state.sensorstime.Mono > 5)  {/*Ahh scremaing we have an inactive sensor*/}  
      if (state.currentTime - state.sensorstime.Sonar > 5)  {/*Ahh scremaing we have an inactive sensor*/}  
    }
      setTopics(renderState.topics);
      setState((oldState) => ({ ...oldState, colorScheme: renderState.colorScheme }));
      
      // Save the most recent message on our topic.
      if (renderState.currentFrame && renderState.currentFrame.length > 0) {
        setMessage(renderState.currentFrame[renderState.currentFrame.length - 1]);
      }
      
    };
    context.watch("currentTime")
    context.watch("topics");
    context.watch("currentFrame");
    context.watch("colorScheme");

  }, [context]);

  // Call our done function at the end of each render.
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  function sleep(ms: any) {
    return new Promise(resolve => setTimeout(resolve, ms));
}
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

        <ReactJson
          name={null}
          src={message}
          indentWidth={2}
          theme={state.colorScheme === "dark" ? "monokai" : "rjv-default"}
          enableClipboard={false}
          displayDataTypes={false}
        />

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
          {state.sensorstime.map((row) => (
            <TableRow
              key={row.name}
              sx={{ '&:last-child td, &:last-child th': { border: 0 } }}
            >
              <TableCell component="th" scope="row">
                {row.name}
              </TableCell>
              <TableCell align="right">{row.calories}</TableCell>
              <TableCell align="right">{row.fat}</TableCell>
              <TableCell align="right">{row.carbs}</TableCell>
              <TableCell align="right">{row.protein}</TableCell>
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
