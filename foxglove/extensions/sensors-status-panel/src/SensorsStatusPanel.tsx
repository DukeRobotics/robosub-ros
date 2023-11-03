import { Immutable, PanelExtensionContext, RenderState, MessageEvent } from "@foxglove/studio";
import Box from "@mui/material/Box";
import Paper from "@mui/material/Paper";
import Table from "@mui/material/Table";
import TableBody from "@mui/material/TableBody";
import TableCell, { tableCellClasses } from "@mui/material/TableCell";
import TableContainer from "@mui/material/TableContainer";
import TableRow from "@mui/material/TableRow";
import Tooltip from "@mui/material/Tooltip";
import { useTheme } from "@mui/material/styles";
import { useLayoutEffect, useEffect, useState, useCallback } from "react";
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
  sensorstime?: SensorsTime;
  connectStatus?: ConnectStatus;
  currentTime?: number;
};

function SensorsStatusPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  const defaultState = useCallback(() => {
    const initialState = {} as State;
    initialState.sensorstime = {
      DVL: 0,
      IMU: 0,
      Depth: 0,
      DepthAI: 0,
      Mono: 0,
      Sonar: 0,
    };
    initialState.currentTime = 1;
    initialState.connectStatus = {
      DVL: false,
      IMU: false,
      Depth: false,
      DepthAI: false,
      Mono: false,
      Sonar: false,
    };
    return initialState;
  }, []);

  // Restore our state from the layout via the context.initialState property.
  const [state, setState] = useState<State>(() => {
    return defaultState();
  });

  useEffect(() => {
    // Make a list of all topics [{topic: topic1}, {topic: topic2 }]
    const topicList: { topic: string }[] = [];

    for (const value of Object.values(TOPICS_DICT)) {
      topicList.push({ topic: value });
    }
    // Subscribe to all topics
    context.subscribe(topicList);
  }, [context]);

  // Setup our onRender function and start watching topics and currentFrame for messages.
  useLayoutEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done: unknown) => {
      setRenderDone(() => done);

      if (renderState.didSeek ?? false) {
        setState(defaultState());
      }

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
    };

    context.watch("currentTime");
    context.watch("topics");
    context.watch("currentFrame");
    context.watch("didSeek");
  }, [context, defaultState, state]);

  // Call our done function at the end of each render.
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // Create a table of all the sensors and their status with the goal of being put into a Table component using a for loop
  const theme = useTheme();
  return (
    <Box m={1}>
      <div>
        <TableContainer component={Paper}>
          <Table
            size="small"
            sx={{
              [`& .${tableCellClasses.root}`]: {
                borderBottom: "none",
              },
            }}
          >
            <TableBody>
              {Object.entries(TOPICS_DICT).map(([sensor, topic]) => (
                <TableRow
                  key={sensor}
                  style={{
                    backgroundColor:
                      state.connectStatus?.[sensor as keyof typeof TOPICS_DICT] ?? false
                        ? theme.palette.success.main
                        : theme.palette.error.main,
                  }}
                >
                  <TableCell sx={{ color: "white" }}>
                    <Tooltip title={topic} arrow placement="right">
                      <span>{sensor}</span>
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
