import { Immutable, PanelExtensionContext, RenderState, Topic, MessageEvent } from "@foxglove/studio";
import Box from "@mui/material/Box";
import Paper from "@mui/material/Paper";
import Table from "@mui/material/Table";
import TableBody from "@mui/material/TableBody";
import TableCell from "@mui/material/TableCell";
import TableContainer from "@mui/material/TableContainer";
import TableRow from "@mui/material/TableRow";
import Tooltip from "@mui/material/Tooltip";
import { useLayoutEffect, useEffect, useState, useMemo } from "react";
import { createRoot } from "react-dom/client";

const TOPICS_DICT = {
  DVL: "/sensors/dvl/odom",
  IMU: "/vectornav/IMU",
  Depth: "/sensors/depth",
  DepthAI: "/camera/front/rgb/preview/compressed",
  Mono: "/camera/usb_camera/compressed",
  Sonar: "/sonar/status",
};

// Reversed dictionary of topics_dict
const topicsDictReversed: { [key: string]: string } = {};
for (const [key, value] of Object.entries(TOPICS_DICT)) {
  topicsDictReversed[value] = key;
}
const TIME_THRESHOLD = 1; // Seconds until sensor is considered disconnected

type SensorsTime = Record<keyof typeof TOPICS_DICT, number>;
type ConnectStatus = Record<keyof typeof TOPICS_DICT, boolean>;

type State = {
  topic?: string;
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

    // Make a list of all topics [{topic: topic1}, {topic: topic2 }]
    const topicList: { topic: string }[] = [];

    for (const [_, value] of Object.entries(TOPICS_DICT)) {
      topicList.push({ topic: value });
    }
    // Subscribe to all topics
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
      setRenderDone(() => done);

      // Updates CurrentTime to the current time
      if (state.currentTime != null && state.currentTime >= 0) {
        state.currentTime = renderState.currentTime?.sec;
      }

      // If sensorstime exists and the current frame exists (onRender was ran due to currentFrame changing)
      if (
        state.currentTime != null &&
        state.sensorstime &&
        state.connectStatus &&
        renderState.currentFrame &&
        renderState.currentFrame.length !== 0
      ) {
        // Define the last frame
        const lastFrame = renderState.currentFrame[renderState.currentFrame.length - 1] as MessageEvent<never>;

        try {
          // Force sensorName to not be undefined
          const sensorName = topicsDictReversed[lastFrame.topic] as keyof typeof TOPICS_DICT;
          state.sensorstime[sensorName] = state.currentTime;
          state.connectStatus[sensorName] = true;
        } catch (error) {
          console.log(error);
        }
      }

      if (state.connectStatus && state.sensorstime && state.currentTime != null) {
        // Compare current time to each sensorstime attribute
        for (const key in TOPICS_DICT) {
          if (state.currentTime - state.sensorstime[key as keyof typeof TOPICS_DICT] > TIME_THRESHOLD) {
            state.connectStatus[key as keyof typeof TOPICS_DICT] = false;
          }
        }
      }

      setTopics(renderState.topics);
    };

    context.watch("currentTime");
    context.watch("topics");
    context.watch("currentFrame");
  }, [context, state]);

  // Call our done function at the end of each render.
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // Create a table of all the sensors and their status with the goal of being put into a Table component using a for loop
  return (
    <Box m={1}>
      <div>
        <TableContainer component={Paper}>
          <Table size="small">
            <TableBody>
              {Object.entries(TOPICS_DICT).map(([sensor, topic]) => (
                <TableRow
                  key={sensor}
                  style={{
                    backgroundColor:
                      state.connectStatus?.[sensor as keyof typeof TOPICS_DICT] ?? false ? "green" : "red",
                  }}
                  sx={{ "&:last-child td, &:last-child th": { border: 0 } }}
                >
                  <TableCell sx={{ color: "white" }}>
                    <Tooltip title={topic} arrow placement="right">
                      <b>{sensor}</b>
                    </Tooltip>
                  </TableCell>
                </TableRow>
              ))}
            </TableBody>
          </Table>
        </TableContainer>
      </div>
    </Box>
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
