import { drcTheme } from "@duke-robotics/theme";
import { Immutable, PanelExtensionContext, RenderState, MessageEvent } from "@foxglove/studio";
import { Typography, useTheme } from "@mui/material";
import Box from "@mui/material/Box";
import Paper from "@mui/material/Paper";
import Table from "@mui/material/Table";
import TableBody from "@mui/material/TableBody";
import TableCell from "@mui/material/TableCell";
import TableContainer from "@mui/material/TableContainer";
import TableRow from "@mui/material/TableRow";
import Tooltip from "@mui/material/Tooltip";
import { ThemeProvider } from "@mui/material/styles";
import { useLayoutEffect, useEffect, useState } from "react";
import { createRoot } from "react-dom/client";

// Map of sensor name to topic name
const TOPICS_MAP = {
  DVL: "/sensors/dvl/odom",
  IMU: "/vectornav/IMU",
  Pressure: "/sensors/depth",
  DepthAI: "/camera/front/rgb/preview/compressed",
  Mono: "/camera/usb_camera/compressed",
  Sonar: "/sonar/status",
};
const SENSOR_DOWN_THRESHOLD = 1; // Seconds until sensor is considered disconnected

const TOPICS_MAP_REVERSED: Record<string, keyof typeof TOPICS_MAP> = {};
for (const [key, value] of Object.entries(TOPICS_MAP)) {
  TOPICS_MAP_REVERSED[value] = key as keyof typeof TOPICS_MAP;
}

type SensorsTime = Record<keyof typeof TOPICS_MAP, number>; // Time of last message received from sensor
type ConnectStatus = Record<keyof typeof TOPICS_MAP, boolean>; // True if SensorsTime is within SENSOR_DOWN_THRESHOLD seconds

interface State {
  sensorsTime: SensorsTime;
  connectStatus: ConnectStatus;
  currentTime: number;
}

const initState = () => {
  const state: Partial<State> = {};

  // Initialize sensorsTime with 0's
  const sensorsTime: Partial<SensorsTime> = {};
  for (const key of Object.keys(TOPICS_MAP)) {
    sensorsTime[key as keyof SensorsTime] = 0;
  }
  state.sensorsTime = sensorsTime as SensorsTime;

  // Initialize connectStatus with false's
  const connectStatus: Partial<ConnectStatus> = {};
  for (const key of Object.keys(TOPICS_MAP)) {
    connectStatus[key as keyof ConnectStatus] = false;
  }
  state.connectStatus = connectStatus as ConnectStatus;

  // Initialize currentTime with Infinity
  // This ensures that (currentTime - sensorsTime > SENSOR_DOWN_THRESHOLD) so that the sensor is initially considered disconnected
  state.currentTime = Infinity;

  return state as State;
};

function SensorsStatusPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [state, setState] = useState<State>(initState());

  // Array of all topics: [{topic: topic1}, {topic: topic2}, ... ]
  const topicList = [];
  for (const value of Object.values(TOPICS_MAP)) {
    topicList.push({ topic: value });
  }
  // Subscribe to all topics
  context.subscribe(topicList);

  // Watch currentFrame for messages from each sensor
  useLayoutEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done: unknown) => {
      setRenderDone(() => done);

      // Reset state when the user seeks the video
      if (renderState.didSeek ?? false) {
        setState(initState());
      }

      // Updates currentTime
      if (renderState.currentTime != undefined) {
        setState((prevState) => ({
          ...prevState,
          currentTime: renderState.currentTime!.sec,
        }));
      }

      if (renderState.currentFrame && renderState.currentFrame.length !== 0) {
        const lastFrame = renderState.currentFrame[renderState.currentFrame.length - 1] as MessageEvent;
        const sensorName = TOPICS_MAP_REVERSED[lastFrame.topic] as string;

        // Update sensorsTime to the current time and set connectStatus to true
        setState((prevState) => ({
          ...prevState,
          sensorsTime: {
            ...prevState.sensorsTime,
            [sensorName]: state.currentTime,
          },
          connectStatus: {
            ...prevState.connectStatus,
            [sensorName]: true,
          },
        }));
      }

      // Compare current time to each sensorsTime and set connectStatus to false if the sensor is down
      for (const key in TOPICS_MAP) {
        if (state.currentTime - state.sensorsTime[key as keyof typeof TOPICS_MAP] > SENSOR_DOWN_THRESHOLD) {
          setState((prevState) => ({
            ...prevState,
            connectStatus: {
              ...prevState.connectStatus,
              [key as keyof typeof TOPICS_MAP]: false,
            },
          }));
        }
      }
    };

    context.watch("currentTime");
    context.watch("currentFrame");
    context.watch("didSeek");
  }, [context, state]);

  // Call our done function at the end of each render.
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // Create a table of all the sensors and their status
  const theme = useTheme();
  return (
    <ThemeProvider theme={drcTheme}>
      <Box m={1}>
        <TableContainer component={Paper}>
          <Table size="small">
            <TableBody>
              {Object.entries(TOPICS_MAP).map(([sensor, topic]) => (
                <TableRow
                  key={sensor}
                  style={{
                    backgroundColor: state.connectStatus[sensor as keyof typeof TOPICS_MAP]
                      ? theme.palette.success.main
                      : theme.palette.error.main,
                  }}
                >
                  <TableCell>
                    <Tooltip title={topic} arrow placement="right">
                      <Typography variant="subtitle2" color={theme.palette.common.white}>
                        {sensor}
                      </Typography>
                    </Tooltip>
                  </TableCell>
                </TableRow>
              ))}
            </TableBody>
          </Table>
        </TableContainer>
      </Box>
    </ThemeProvider>
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
