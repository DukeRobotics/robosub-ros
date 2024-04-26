import useTheme from "@duke-robotics/theme";
import { Immutable, PanelExtensionContext, RenderState, MessageEvent, Subscription } from "@foxglove/studio";
import { Box, Paper, Table, TableBody, TableCell, TableContainer, TableRow, Typography, Tooltip } from "@mui/material";
import { ThemeProvider } from "@mui/material/styles";
import React, { useEffect, useState } from "react";
import { createRoot } from "react-dom/client";

// Map of sensor name to topic name
const TOPICS_MAP = {
  DVL: "/sensors/dvl/raw",
  IMU: "/vectornav/IMU",
  Pressure: "/sensors/depth",
  "Front Camera": "/camera/front/rgb/preview/compressed",
  Mono: "/camera/usb_camera/compressed",
  Sonar: "/sonar/status",
  Ping1D: "/sensors/ping/distance",
};

type topicsMapKeys = keyof typeof TOPICS_MAP;

// Seconds until sensor is considered disconnected
const SENSOR_DOWN_THRESHOLD = 1;

const TOPICS_MAP_REVERSED: Record<string, topicsMapKeys> = {};
for (const [key, value] of Object.entries(TOPICS_MAP)) {
  TOPICS_MAP_REVERSED[value] = key as topicsMapKeys;
}

// Array of all topics: [{topic: topic1}, {topic: topic2}, ... ]

const TOPICS_LIST: Subscription[] = [];
for (const value of Object.values(TOPICS_MAP)) {
  TOPICS_LIST.push({ topic: value });
}

// Time of last message received from sensor
type SensorsTime = Record<topicsMapKeys, number>;
// True if SensorsTime is within SENSOR_DOWN_THRESHOLD seconds
type ConnectStatus = Record<topicsMapKeys, boolean>;

type SensorsStatusPanelState = {
  sensorsTime: SensorsTime;
  connectStatus: ConnectStatus;
  currentTime: number;
};

const initState = () => {
  const state: Partial<SensorsStatusPanelState> = {};

  // Initialize sensorsTime with 0's
  const sensorsTime: Partial<SensorsTime> = {};
  for (const key in TOPICS_MAP) {
    sensorsTime[key as keyof SensorsTime] = 0;
  }
  state.sensorsTime = sensorsTime as SensorsTime;

  // Initialize connectStatus with false's
  const connectStatus: Partial<ConnectStatus> = {};
  for (const key in TOPICS_MAP) {
    connectStatus[key as keyof ConnectStatus] = false;
  }
  state.connectStatus = connectStatus as ConnectStatus;

  // Initialize currentTime with Infinity
  // This ensures that (currentTime - sensorsTime > SENSOR_DOWN_THRESHOLD) so that the sensor is initially considered disconnected
  state.currentTime = Infinity;

  return state as SensorsStatusPanelState;
};

function SensorsStatusPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [state, setState] = useState<SensorsStatusPanelState>(initState());

  // Subscribe to all topics
  context.subscribe(TOPICS_LIST);

  // Watch currentFrame for messages from each sensor
  useEffect(() => {
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
        const lastFrame = renderState.currentFrame.at(-1) as MessageEvent;
        const sensorName = TOPICS_MAP_REVERSED[lastFrame.topic] as string;

        // Update sensorsTime to the current time and set connectStatus to true
        setState((prevState) => ({
          ...prevState,
          sensorsTime: {
            ...prevState.sensorsTime,
            [sensorName]: prevState.currentTime,
          },
          connectStatus: {
            ...prevState.connectStatus,
            [sensorName]: true,
          },
        }));
      }

      // Compare current time to each sensorsTime and set connectStatus to false if the sensor is down
      for (const key in TOPICS_MAP) {
        if (state.currentTime - state.sensorsTime[key as topicsMapKeys] > SENSOR_DOWN_THRESHOLD) {
          setState((prevState) => ({
            ...prevState,
            connectStatus: {
              ...prevState.connectStatus,
              [key as topicsMapKeys]: false,
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
    <ThemeProvider theme={theme}>
      <Box m={1}>
        <TableContainer component={Paper}>
          <Table size="small">
            <TableBody>
              {Object.entries(TOPICS_MAP).map(([sensor, topic]) => (
                <TableRow
                  key={sensor}
                  style={{
                    backgroundColor: state.connectStatus[sensor as topicsMapKeys]
                      ? theme.palette.success.dark
                      : theme.palette.error.dark,
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
  context.panelElement.style.overflow = "auto"; // Enable scrolling

  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<SensorsStatusPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
