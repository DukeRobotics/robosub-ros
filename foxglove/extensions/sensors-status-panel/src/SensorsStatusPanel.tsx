/* eslint-disable @typescript-eslint/no-unused-vars */
/* eslint-disable react/no-deprecated */
/* eslint-disable prettier/prettier */
import { Immutable, PanelExtensionContext, RenderState, Topic, MessageEvent } from "@foxglove/studio";
import Alert from "@mui/material/Alert";
import Paper from "@mui/material/Paper";
import Table from "@mui/material/Table";
import TableBody from "@mui/material/TableBody";
import TableCell from "@mui/material/TableCell";
import TableContainer from "@mui/material/TableContainer";
import TableHead from "@mui/material/TableHead";
import TableRow from "@mui/material/TableRow";
import Tooltip from "@mui/material/Tooltip";
import * as React from "react";
import { useLayoutEffect, useEffect, useState, useRef, useMemo } from "react";
import * as ReactDOM from "react-dom";

const topics_dict = {
  DVL: "/sensors/dvl/odom",
  IMU: "/vectornav/IMU",
  Depth: "/sensors/depth",
  DepthAI: "/camera/front/rgb/preview/compressed",
  Mono: "/camera/usb_camera/compressed",
  Sonar: "/sonar/status",
};

//Reversed dictionary of topics_dict
const topics_dict_reversed: { [key: string]: string } = {};
for (const [key, value] of Object.entries(topics_dict)) {
  topics_dict_reversed[value] = key;
}

type SensorsTime = {} & Record<keyof typeof topics_dict, number>;
type ConnectStatus = {} & Record<keyof typeof topics_dict, boolean>;

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
    const Initialstate = context.initialState as State;
    Initialstate.sensorstime = {
      DVL: 0,
      IMU: 0,
      Depth: 0,
      DepthAI: 0,
      Mono: 0,
      Sonar: 0,
    };
    Initialstate.currentTime = 1;
    Initialstate.connectStatus = {
      DVL: false,
      IMU: false,
      Depth: false,
      DepthAI: false,
      Mono: false,
      Sonar: false,
    };
    return Initialstate;
  });

  // Get topics
  const imageTopics = useMemo(() => topics ?? [], [topics]);

  useEffect(() => {
    // Save our state to the layout when the topic changes.
    context.saveState({ topic: state.topic });

    //make a list of all topics [{topic: topic1}, {topic: topic2 }]
    const topicList: { topic: string }[] = [];
    for (const [key, value] of Object.entries(topics_dict)) {
      topicList.push({ topic: value });
    }
    //Subscribe to all topics
    context.subscribe(topicList);
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
    context.onRender = (renderState: Immutable<RenderState>, done: any) => {
      // eslint-disable-next-line @typescript-eslint/no-unsafe-return
      setRenderDone(() => done);

      //Updates CurrentTime to the current time
      if (state.currentTime != null && state.currentTime >= 0) {
        state.currentTime = renderState.currentTime?.sec;
      }

      //If sensorstime exists and the current frame exists (onRender was ran due to currentFrame changing)
      if (
        state.currentTime &&
        state.sensorstime &&
        state.connectStatus &&
        renderState.currentFrame &&
        renderState.currentFrame.length !== 0
      ) {
        //Define the last frame
        const lastFrame = renderState.currentFrame[renderState.currentFrame.length - 1] as MessageEvent<any>;

        //Switch statements on the topic of the last frame

        if (lastFrame) {
          // try catch statement to see if the topic is in the dictionary
          //force lastFrame.topic to not be undefined

          try {
            //force sensorName to not be undefined
            const sensorName = topics_dict_reversed[lastFrame.topic] as keyof typeof topics_dict;
            state.sensorstime[sensorName] = state.currentTime;
            state.connectStatus[sensorName] = true;
          } catch (error) {
            console.log(error);
          }
        }
      }

      if (state.connectStatus && state.sensorstime && state.currentTime) {
        //Compare current time to each sensorstime attribute
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

  //create a table of all the sensors and their status with the goal of being put into a Table component using a for loop
  return (
    <div style={{ height: "100%", padding: "1rem" }}>
      <div>
        <TableContainer component={Paper}>
          <Table size="small" aria-label="simple tble">
            <TableBody>
              {Object.entries(topics_dict).map(([sensor, topic]) => (
                // eslint-disable-next-line @typescript-eslint/strict-boolean-expressions
                <TableRow
                  key={sensor}
                  style={{
                    backgroundColor: state.connectStatus?.[sensor as keyof typeof topics_dict] ? "green" : "red", }}
                  sx={{ "&:last-child td, &:last-child th": { border: 0 } }}
                >
                  <Tooltip title={topic} arrow>
                    <TableCell sx={{ color: "white" }}>
                      {" "}
                      <b> {sensor} </b>
                    </TableCell>
                  </Tooltip>
                </TableRow>
              ))}
            </TableBody>
          </Table>
        </TableContainer>
      </div>
    </div>
  );
}

function createData(Sensor_Name: string, Topic_Path: string, Sensor_Publishing: boolean) {
  return { Sensor_Name, Topic_Path, Sensor_Publishing };
}

export function initSensorsStatusPanel(context: PanelExtensionContext): () => void {
  ReactDOM.render(<SensorsStatusPanel context={context} />, context.panelElement);

  // Return a function to run when the panel is removed
  return () => {
    ReactDOM.unmountComponentAtNode(context.panelElement);
  };
}
