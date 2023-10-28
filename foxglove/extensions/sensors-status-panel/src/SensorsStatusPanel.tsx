import { Immutable, PanelExtensionContext, RenderState, Topic, MessageEvent } from "@foxglove/studio";
import Paper from "@mui/material/Paper";
import Table from "@mui/material/Table";
import TableBody from "@mui/material/TableBody";
import TableCell from "@mui/material/TableCell";
import TableContainer from "@mui/material/TableContainer";
import TableRow from "@mui/material/TableRow";
import Tooltip from "@mui/material/Tooltip";
import { useLayoutEffect, useEffect, useState, useMemo } from "react";
import { createRoot } from "react-dom/client";

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

type SensorsTime = NonNullable<unknown> & Record<keyof typeof topics_dict, number>;
type ConnectStatus = NonNullable<unknown> & Record<keyof typeof topics_dict, boolean>;

type State = {
  topic?: string;
  colorScheme?: RenderState["colorScheme"];
  sensorstime?: SensorsTime;
  connectStatus?: ConnectStatus;
  currentTime?: number;
};

function SensorsStatusPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [topics, setTopics] = useState<readonly Topic[] | undefined>();

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
    // eslint-disable-next-line @typescript-eslint/no-unused-vars
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

  // Setup our onRender function and start watching topics and currentFrame for messages.
  useLayoutEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done: unknown) => {
      // eslint-disable-next-line @typescript-eslint/no-unsafe-return
      setRenderDone(() => done);

      //Updates CurrentTime to the current time
      if (state.currentTime != null && state.currentTime >= 0) {
        state.currentTime = renderState.currentTime?.sec;
      }

      //If sensorstime exists and the current frame exists (onRender was ran due to currentFrame changing)
      if (
        state.currentTime != null &&
        state.sensorstime &&
        state.connectStatus &&
        renderState.currentFrame &&
        renderState.currentFrame.length !== 0
      ) {
        //Define the last frame
        const lastFrame = renderState.currentFrame[renderState.currentFrame.length - 1] as MessageEvent<never>;

        //Switch statements on the topic of the last frame

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

      if (state.connectStatus && state.sensorstime && state.currentTime != null) {
        //Compare current time to each sensorstime attribute
        for (const key in topics_dict) {
          if (state.currentTime - state.sensorstime[key as keyof typeof topics_dict] > 1) {
            state.connectStatus[key as keyof typeof topics_dict] = false;
          }
        }
      }

      setTopics(renderState.topics);
      setState((oldState) => ({ ...oldState, colorScheme: renderState.colorScheme }));
    };
    context.watch("currentTime");
    context.watch("topics");
    context.watch("currentFrame");
    context.watch("colorScheme");
  }, [context, state]);

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
                    backgroundColor:
                      state.connectStatus?.[sensor as keyof typeof topics_dict] ?? false ? "green" : "red",
                  }}
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

export function initSensorsStatusPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<SensorsStatusPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
