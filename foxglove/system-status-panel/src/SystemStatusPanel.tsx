import { PanelExtensionContext, RenderState, Topic, MessageEvent } from "@foxglove/studio";
import { useLayoutEffect, useEffect, useState, useRef, useMemo } from "react";
import ReactDOM from "react-dom";
import ReactJson from "react-json-view";
import Alert from '@mui/material/Alert';

import * as React from 'react';
import Table from '@mui/material/Table';
import TableBody from '@mui/material/TableBody';
import TableCell from '@mui/material/TableCell';
import TableContainer from '@mui/material/TableContainer';
import TableHead from '@mui/material/TableHead';
import TableRow from '@mui/material/TableRow';
import Paper from '@mui/material/Paper';

function createData(
  Status: string,
  Value: number,
) {
  return { Status, Value };
}

type State = {
  topic?: string;
  colorScheme?: RenderState["colorScheme"];
  cpuUsage?;
  ramUsage?;
};

function SystemStatusPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [topics, setTopics] = useState<readonly Topic[] | undefined>();
  const [message, setMessage] = useState<any>();

  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  // Restore our state from the layout via the context.initialState property.
  const [state, setState] = useState<State>(() => {
    const initialState = context.initialState as State;
    initialState.topic = '/system/usage';
    return initialState;
  });

  // Get topics
  const imageTopics = useMemo(
    () => (topics ?? []),
    [topics],
  );

  //define values in table
  const rows = [
    createData('CPU', state.cpuUsage),
    createData('RAM', state.ramUsage),
    createData('Voltage', 0), //TODO: enter voltage remaining
  ];
  
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

  // Setup our onRender function and start watching topics and currentFrame for messages.
  useLayoutEffect(() => {
    context.onRender = (renderState: RenderState, done) => {
      setRenderDone(() => done);
      setTopics(renderState.topics);
      setState((oldState) => ({ ...oldState, colorScheme: renderState.colorScheme }));
      
      // Updating CPU/RAM/Voltage data.
      if (renderState.currentFrame && renderState.currentFrame.length > 0) {
        const latestFrame = renderState.currentFrame[renderState.currentFrame.length - 1] as MessageEvent<any>;
        setState((oldState) => ({ ...oldState,
                                  cpuUsage : latestFrame?.message?.cpu_percent,
                                  ramUsage : latestFrame?.message?.ram.percentage}))
        console.log(latestFrame?.message?.["cpu_percent"]);
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

  return (
    <div style={{ height: "100%", padding: "1rem" }}>
      <h2>System Status</h2>
      {context.subscribe == undefined && (
        <Alert variant="filled" severity="error">Subscribing to topics is not supported by this connection</Alert>
      )}
      <div>
        {/* <label>Choose a topic to display: </label>
        
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
        </select> */}
        

        <ReactJson
          name={null}
          src={message}
          indentWidth={2}
          theme={state.colorScheme === "dark" ? "monokai" : "rjv-default"}
          enableClipboard={false}
          displayDataTypes={false}
        />

        <TableContainer component={Paper}>
              <Table sx={{ minWidth: 100 }} aria-label="simple table">
                <TableHead>
                  <TableRow>
                    <TableCell>Component</TableCell>
                    <TableCell align="right">Usage</TableCell>
                    
                  </TableRow>
                </TableHead>
                <TableBody>
                  {rows.map((row) => (
                    <TableRow
                      key={row.Status}
                      sx={{ '&:last-child td, &:last-child th': { border: 0 } }}
                    >
                      <TableCell component="th" scope="row">
                        {row.Status}
                      </TableCell>
                      <TableCell align="right">{row.Value}</TableCell>
                    </TableRow>
                  ))}
                </TableBody>
              </Table>
            </TableContainer>

            </div>
          </div>

    
  );
}

export function initSystemStatusPanel(context: PanelExtensionContext): () => void {
  ReactDOM.render(<SystemStatusPanel context={context} />, context.panelElement);

  // Return a function to run when the panel is removed
  return () => {
    ReactDOM.unmountComponentAtNode(context.panelElement);
  };
}
