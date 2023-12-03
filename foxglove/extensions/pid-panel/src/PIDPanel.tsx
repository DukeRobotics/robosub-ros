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
  Grid,
} from "@mui/material";
import { ThemeProvider, createTheme } from "@mui/material/styles";
import React, { useEffect, useLayoutEffect, useState } from "react";
import { JSX } from "react/jsx-runtime";
import { createRoot } from "react-dom/client";

import {
  CustomMsgsPidGainConst,
  CustomMsgsPidGains,
  CustomMsgsSetPidGainsRequest,
  CustomMsgsSetPidGainsResponse,
} from "./types";

type PIDPanelState = {
  serviceName: string;
  request: string;
  response?: CustomMsgsSetPidGainsResponse;
  error?: Error | undefined;
  colorScheme?: RenderState["colorScheme"];
  panelMode: CustomMsgsPidGainConst.LOOP_POSITION | CustomMsgsPidGainConst.LOOP_VELOCITY;
  message?: MessageEvent<CustomMsgsPidGains>;
  focusStatus: Record<number, Record<number, boolean>>;
  editedValues: Record<number, Record<number, number>>;
};

// Triple nested dictionary to get PID values
const defaultPid = () => {
  const gains: Record<number, number> = {
    [CustomMsgsPidGainConst.GAIN_KP]: -1, // GAIN_KP
    [CustomMsgsPidGainConst.GAIN_KI]: -1, // GAIN_KI
    [CustomMsgsPidGainConst.GAIN_KD]: -1, // GAIN_KD
    [CustomMsgsPidGainConst.GAIN_FF]: -1, // GAIN_KF
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

  return pid;
};
const pid = defaultPid();

const READ_PID_TOPIC = "/controls/pid_gains";
const SET_PID_SERVICE = "/controls/set_pid_gains";

function PIDPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [state, setState] = useState<PIDPanelState>({
    serviceName: SET_PID_SERVICE,
    request: "{}",
    panelMode: CustomMsgsPidGainConst.LOOP_POSITION,
    focusStatus: {},
    editedValues: {},
  });

  context.subscribe([{ topic: READ_PID_TOPIC }]);

  useLayoutEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setRenderDone(() => done);
      setState((oldState) => ({ ...oldState, colorScheme: renderState.colorScheme }));

      if (renderState.currentFrame && renderState.currentFrame.length > 0) {
        const lastFrame = renderState.currentFrame.at(-1) as MessageEvent<CustomMsgsPidGains>;
        // Loop through lastFrame and update PID values
        const pidGains = lastFrame.message.pid_gains;
        for (const pidGain of pidGains) {
          pid[pidGain.loop]![pidGain.axis] = {
            ...pid[pidGain.loop]![pidGain.axis],
            [pidGain.gain]: pidGain.value,
          };
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
  const callService = async (serviceName: string, request: unknown) => {
    if (!context.callService) {
      return;
    }

    console.log("Calling service", serviceName, request);

    try {
      const response = await context.callService(serviceName, request);
      const typedResponse = response as CustomMsgsSetPidGainsResponse;
      setState((oldState) => ({
        ...oldState,
        typedResponse,
        error: undefined,
      }));
    } catch (error) {
      setState((oldState) => ({ ...oldState, error: error as Error }));
      console.error(error);
    }
  };

  const handleModeChange = (
    _: React.SyntheticEvent,
    mode: CustomMsgsPidGainConst.LOOP_POSITION | CustomMsgsPidGainConst.LOOP_VELOCITY,
  ) => {
    setState((oldState) => ({ ...oldState, panelMode: mode }));

    handleReset();
  };

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

  const handleFocus = (axis: number, index: number) => {
    setState((prevState) => ({
      ...prevState,
      focusStatus: {
        ...prevState.focusStatus,
        [axis]: {
          ...prevState.focusStatus[axis],
          [index]: true,
        },
      },
    }));
  };

  const handleInputChange = (event: React.ChangeEvent<HTMLInputElement>, axis: number, gain: number) => {
    const isFocused = state.focusStatus[Number(axis)]?.[Number(gain)] ?? false;
    if (isFocused) {
      setState((prevState) => ({
        ...prevState,
        editedValues: {
          ...prevState.editedValues,
          [axis]: {
            ...prevState.editedValues[axis],
            [gain]: Number(event.target.value),
          },
        },
      }));
    }
  };

  const handleReset = () => {
    setState((prevState) => ({
      ...prevState,
      focusStatus: {},
      editedValues: {},
    }));
  };

  const handleSubmit = () => {
    const request: CustomMsgsSetPidGainsRequest = {
      pid_gains: [],
    };
    console.log(state.panelMode);
    Object.entries(state.editedValues).forEach(([axis, gains]) => {
      Object.entries(gains).forEach(([gain, value]) => {
        request.pid_gains.push({
          loop: state.panelMode,
          axis: Number(axis),
          gain: Number(gain),
          value: Number(value),
        });
      });
    });

    void callService(SET_PID_SERVICE, request);

    handleReset();
  };

  return (
    <ThemeProvider theme={theme}>
      <Box m={1}>
        {context.callService == undefined && (
          <Alert variant="filled" severity="error">
            Calling services is not supported by this connection
          </Alert>
        )}
        <Tabs value={state.panelMode} onChange={handleModeChange} variant="fullWidth">
          <Tab label="Position" value={CustomMsgsPidGainConst.LOOP_POSITION} />
          <Tab label="Velocity" value={CustomMsgsPidGainConst.LOOP_VELOCITY} />
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
              <TableBody>
                {pid[state.panelMode] &&
                  Object.entries(pid[state.panelMode] ?? {}).map(([axis, g]) => (
                    <TableRow key={axis}>
                      <TableCell width="10px">{getAxisEnumName(Number(axis))}</TableCell>
                      {Object.values(g).map((gain, index) => {
                        const isFocused = state.focusStatus[Number(axis)]?.[index] ?? false;
                        return (
                          <TableCell key={index} width="10px">
                            <input
                              type="text"
                              style={{
                                width: "100%",
                                boxSizing: "border-box",
                                borderColor: isFocused ? "red" : "",
                              }}
                              value={isFocused ? undefined : gain}
                              onFocus={() => {
                                handleFocus(Number(axis), index);
                              }}
                              onChange={(event) => {
                                handleInputChange(event, Number(axis), index);
                              }}
                            />
                          </TableCell>
                        );
                      })}
                    </TableRow>
                  ))}
              </TableBody>
            </Table>
          </TableContainer>

          <Grid container spacing={2}>
            <Grid item xs={6}>
              <Button variant="contained" fullWidth onClick={handleSubmit}>
                Submit
              </Button>
            </Grid>
            <Grid item xs={6}>
              <Button variant="contained" fullWidth onClick={handleReset}>
                Reset
              </Button>
            </Grid>
          </Grid>
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
