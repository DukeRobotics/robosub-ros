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
  sensortime?: SensorsTime;
};

function SensorsStatusPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [topics, setTopics] = useState<readonly Topic[] | undefined>();
  const [message, setMessage] = useState<any>();

  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  // Restore our state from the layout via the context.initialState property.
  const [state, setState] = useState<State>(() => {
    var Initialstate = context.initialState as State;
    Initialstate.sensortime = {
      DVL: Date.now(), 
      IMU: Date.now(), 
      Depth: Date.now(), 
      DepthAI: Date.now(), 
      Mono: Date.now(), 
      Sonar: Date.now()
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

  var lastRender = Date.now();
  // Setup our onRender function and start watching topics and currentFrame for messages.
  useLayoutEffect(() => {
    context.onRender = (renderState: RenderState, done) => {
      setRenderDone(() => done);

      //If sensortime exists
      if (state.sensortime) {
        //TODO: Add switch statements to check what topic just published 
        state.sensortime.DVL = Date.now()
      }

      

      setTopics(renderState.topics);
      setState((oldState) => ({ ...oldState, colorScheme: renderState.colorScheme }));
      
      // Save the most recent message on our topic.
      if (renderState.currentFrame && renderState.currentFrame.length > 0) {
        setMessage(renderState.currentFrame[renderState.currentFrame.length - 1]);
      }
      
    };

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

(async function checkTimeSinceRender() {
    const waitInSeconds = 1;
    var beginning = Date.now() - 0 //TODO: state.sensortime.SENSO;
    await sleep(waitInSeconds * 1000);
    state.topic 
    
})();
//TODO: Make if statement to see if sensortime exists
const SensorTable = [
  createData('DVL', '/sensors/dvl/odom', ( state.sensortime.DVL ? true : false)),
  createData('IMU','/vectornav/IMU',true),
  createData('Depth', '/sensors/depth',true),
  createData('DepthAI Camera', '/camera/front/rgb/preview/compressed', true),
  createData('Mono Camera','/camera/usb_camera/compressed',true),
  createData('Sonar', '/sonar/status', true),
];

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
