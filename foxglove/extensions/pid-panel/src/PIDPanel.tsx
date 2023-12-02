import { Immutable, PanelExtensionContext, RenderState, MessageEvent } from "@foxglove/studio";
import {
  useMediaQuery,
  Alert,
  Tab,
  Tabs,
  TableBody,
  TableContainer,
  TableCell,
  TableRow,
  TableHead,
  Table,
  Box,
  Button,
} from "@mui/material";
// X import { JsonViewer } from "@textea/json-viewer";
import { ThemeProvider, createTheme } from "@mui/material/styles";
import React, { useEffect, useLayoutEffect, useState } from "react";
import { JSX } from "react/jsx-runtime";
import { createRoot } from "react-dom/client";

import {
  CustomMsgsPidGain,
  CustomMsgsPidGainConst,
  CustomMsgsPidGains,
  CustomMsgsSetPidGainsRequest,
  CustomMsgsSetPidGainsResponse,
} from "./types";


type PIDPanelState = {
  serviceName: string;
  request: string;
  response?: unknown;
  error?: Error | undefined;
  colorScheme?: RenderState["colorScheme"];
  panelMode: PanelMode;
  message?: MessageEvent<CustomMsgsPidGains>;
};

// Triple nested dictionary to get PID values
const gains: Record<number, number> = {
  [CustomMsgsPidGainConst.GAIN_KP]: 2.0, // GAIN_KP
  [CustomMsgsPidGainConst.GAIN_KI]: 0.0, // GAIN_KI
  [CustomMsgsPidGainConst.GAIN_KD]: 0.0, // GAIN_KD
  [CustomMsgsPidGainConst.GAIN_FF]: 0.0, // GAIN_KF
};

const axes: Record<number, Record<number, number>> = {
  [CustomMsgsPidGainConst.AXIS_X]: { ...gains }, // X
  [CustomMsgsPidGainConst.AXIS_Y]: { ...gains }, // Y
  [CustomMsgsPidGainConst.AXIS_Z]: { ...gains }, // Z
  [CustomMsgsPidGainConst.AXIS_ROLL]: { ...gains }, // Roll
  [CustomMsgsPidGainConst.AXIS_PITCH]: { ...gains }, // Pitch
  [CustomMsgsPidGainConst.AXIS_YAW]: { ...gains }, // Yaw
};
const pid: Record<number, Record<number, Record<number, number>>> = {
  [CustomMsgsPidGainConst.LOOP_POSITION]: { ...axes }, // Loop Position
  [CustomMsgsPidGainConst.LOOP_VELOCITY]: { ...axes }, // Loop Velocity
};

const topicName = "/current_pid";
const serviceName = "/set_pid";
const pidTypeToUpdate = CustomMsgsPidGainConst.LOOP_POSITION;

enum PanelMode {
  SUBSCRIBING,
  EDITING,
}

function submitGains(pos = 0, axis, gain, newValue) {
  
}

function PIDPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [state, setState] = useState<PIDPanelState>({
    serviceName,
    request: "{}",
    panelMode: PanelMode.SUBSCRIBING,
  });

  context.subscribe([{ topic: topicName }]);

  useLayoutEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setRenderDone(() => done);
      setState((oldState) => ({ ...oldState, colorScheme: renderState.colorScheme }));

      if (renderState.currentFrame && renderState.currentFrame.length > 0) {
        const lastFrame = renderState.currentFrame.at(-1) as MessageEvent<CustomMsgsPidGains>;
        // Loop through lastFrame and update PID values
        const pidGains = lastFrame.message.pid_gains;
        for (const pidGain of pidGains) {
          console.log(pidGain);
          pid[pidGain.loop]![pidGain.axis] = {
            ...pid[pidGain.loop]![pidGain.axis],
            [pidGain.gain]: pidGain.value,
          };
          console.log(pidGain.value);
          console.log(pid);
          console.log("Loop", pidGain.loop);
          console.log("Axis", pidGain.axis);
          console.log("Gain", pidGain.gain);

          console.log(pid[pidGain.loop]![pidGain.axis]![pidGain.gain]);
        }
        setState((oldState) => ({ ...oldState, message: lastFrame }));
      }
    };
  }, [context]);
  context.watch("colorScheme");
  context.watch("currentFrame");

  // Call our done function at the end of each render
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // Call a service with a given request
  // const callService = async (serviceName: string, request: string) => {
  //   if (!context.callService) {
  //     return;
  //   }

  //   try {
  //     const response = await context.callService(serviceName, JSON.parse(request));
  //     JSON.stringify(response); // Attempt serializing the response, to throw an error on failure
  //     setState((oldState) => ({
  //       ...oldState,
  //       response,
  //       error: undefined,
  //     }));
  //   } catch (error) {
  //     setState((oldState) => ({ ...oldState, error: error as Error }));
  //     console.error(error);
  //   }
  // };

  const handleModeChange = (_: React.SyntheticEvent, mode: PanelMode) => {
    setState((oldState) => ({ ...oldState, panelMode: mode }));
  };

  // Close callService with the current state for use in the button
  // Const callServiceWithRequest = () => {
  //   Void callService(state.serviceName, state.request);
  // };

  // TODO: Import this theme once thruster-speeds-panel is merged
  const prefersDarkMode = useMediaQuery("(prefers-color-scheme: dark)");
  const theme = createTheme({
    palette: {
      mode: prefersDarkMode ? "dark" : "light",
    },
    components: {
      MuiTableRow: {
        styleOverrides: {
          root: {
            "&:last-child td, &:last-child th": {
              border: 0,
            },
          },
        },
      },
    },
  });

  // TODO: Maybe refactor this
  function getAxisEnumName(enumValue: number): string {
    const filteredEnumEntries = Object.entries(CustomMsgsPidGainConst).filter(([key, value]) => {
      // eslint-disable-next-line @typescript-eslint/no-unsafe-enum-comparison
      return key.startsWith("AXIS") && value === enumValue;
    });

    if (filteredEnumEntries.length > 0 && filteredEnumEntries[0] != null) {
      return filteredEnumEntries[0][0];
    } else {
      return "Unknown";
    }
  }

  return (
    <ThemeProvider theme={theme}>
      <Box m={1}>
        {context.callService == undefined && (
          <Alert variant="filled" severity="error">
            Calling services is not supported by this connection
          </Alert>
        )}
        <Tabs value={state.panelMode} onChange={handleModeChange} variant="fullWidth">
          <Tab label="Subscribing" value={PanelMode.SUBSCRIBING} />
          <Tab label="Editing" value={PanelMode.EDITING} />
        </Tabs>
        <div>
          <TableContainer>
            <Table size="small" aria-label="simple table">
              <TableHead>
                <TableRow>
                  <TableCell align="center" width="10px"></TableCell>
                  <TableCell align="center" width="10px">
                    Gain_KP
                  </TableCell>
                  <TableCell align="center" width="10px">
                    Gain_KI
                  </TableCell>
                  <TableCell align="center" width="10px">
                    Gain_KD
                  </TableCell>
                  <TableCell align="center" width="10px">
                    Gain_FF
                  </TableCell>
                </TableRow>
              </TableHead>
              {state.panelMode === PanelMode.SUBSCRIBING ? (
                // Table for Subscribing
                <TableBody>
                  {pid[pidTypeToUpdate] &&
                    Object.entries(pid[pidTypeToUpdate]).map(([axis, g]) => (
                      <TableRow key={axis}>
                        <TableCell width="10px">{getAxisEnumName(Number(axis))}</TableCell>
                        {Object.values(g).map((gain, index) => (
                          <TableCell key={index} width="10px">
                            {gain}
                          </TableCell>
                        ))}
                      </TableRow>
                    ))}
                </TableBody>
              ) : (
                // Editing
                <TableBody>
                  {pid[pidTypeToUpdate] &&
                    Object.entries(pid[pidTypeToUpdate]).map(([axis, g]) => (
                      <TableRow key={axis}>
                        <TableCell width="10px">{getAxisEnumName(Number(axis))}</TableCell>
                        {Object.values(g).map((gain, index) => (
                          <TableCell width="10px" key={index}>
                            {gain} <input type="text" id="fname" name={String([axis, g])} size={1}></input>
                          </TableCell>
                        ))}
                      </TableRow>
                    ))}
                </TableBody>
              )}
            </Table>
          </TableContainer>
          
          <form id="updateform">
            <input type="submit" value="Update"></input>
          </form>
        </div>
      </Box>
    </ThemeProvider>
  );
}

export function initPIDPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<PIDPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}

/*


Const updatePID = (event: React.ChangeEvent<HTMLInputElement>) => {
  let hasError = false;
  const value = event.target.value;

  if (value !== "" && (Number.isNaN(Number(value)) || parseInt(value) !== parseFloat(value))) {
    hasError = true;
  } else {
      pid.forEach((thruster: keyof pid) => {
      const speed: number | "" =
        thruster !== event.target.id ? state.tempThrusterSpeeds[thruster] : value !== "" ? parseInt(value) : "";
      if (!validateInput(speed)) {
        hasError = true;
      }
    });
  }

  */
