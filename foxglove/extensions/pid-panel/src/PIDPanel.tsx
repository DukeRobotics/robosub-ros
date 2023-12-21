import {
  CustomMsgsPidGainConst,
  CustomMsgsPidGains,
  CustomMsgsSetPidGainsRequest,
  CustomMsgsSetPidGainsResponse,
} from "@duke-robotics/defs/types";
import useTheme from "@duke-robotics/theme";
import { Immutable, PanelExtensionContext, RenderState, MessageEvent } from "@foxglove/studio";
import {
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
  TextField,
} from "@mui/material";
import { ThemeProvider } from "@mui/material/styles";
import React, { useEffect, useLayoutEffect, useState } from "react";
import { JSX } from "react/jsx-runtime";
import { createRoot } from "react-dom/client";

const PID_TOPIC = "/controls/pid_gains";
const SET_PID_SERVICE = "/controls/set_pid_gains";

type PIDPanelState = {
  error?: Error;
  loopType: CustomMsgsPidGainConst.LOOP_POSITION | CustomMsgsPidGainConst.LOOP_VELOCITY;
  editedValues: Record<number, Record<number, number>>; // Values that will be submitted to the service
};

// Triple nested object to store current PID gains
const initPid = () => {
  // Initialize gains to -1 to signify that they have not been received yet
  const gains: Record<number, number> = {
    [CustomMsgsPidGainConst.GAIN_KP]: -1,
    [CustomMsgsPidGainConst.GAIN_KI]: -1,
    [CustomMsgsPidGainConst.GAIN_KD]: -1,
    [CustomMsgsPidGainConst.GAIN_FF]: -1,
  };
  const axes: Record<number, Record<number, number>> = {
    [CustomMsgsPidGainConst.AXIS_X]: { ...gains },
    [CustomMsgsPidGainConst.AXIS_Y]: { ...gains },
    [CustomMsgsPidGainConst.AXIS_Z]: { ...gains },
    [CustomMsgsPidGainConst.AXIS_ROLL]: { ...gains },
    [CustomMsgsPidGainConst.AXIS_PITCH]: { ...gains },
    [CustomMsgsPidGainConst.AXIS_YAW]: { ...gains },
  };
  const loops: Record<number, Record<number, Record<number, number>>> = {
    [CustomMsgsPidGainConst.LOOP_POSITION]: { ...axes },
    [CustomMsgsPidGainConst.LOOP_VELOCITY]: { ...axes },
  };

  return loops;
};
const pid = initPid();

function PIDPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [state, setState] = useState<PIDPanelState>({
    loopType: CustomMsgsPidGainConst.LOOP_POSITION,
    editedValues: {},
    error: undefined,
  });

  context.subscribe([{ topic: PID_TOPIC }]);

  useLayoutEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setRenderDone(() => done);
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
      }
    };
  }, [context]);
  context.watch("currentFrame");

  // Call our done function at the end of each render
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // Call a service with a given request
  const callService = (serviceName: string, request: unknown) => {
    if (!context.callService) {
      return;
    }

    context.callService(serviceName, request).then(
      (response) => {
        const typedResponse = response as CustomMsgsSetPidGainsResponse;

        // Update the state based on the service response
        // If the service responds with failure, display the response message as an error
        const error = typedResponse.success ? undefined : Error(typedResponse.message);
        setState((oldState) => ({
          ...oldState,
          error,
        }));
      },
      (error) => {
        // Handle service call errors (e.g., service is not advertised)
        setState((oldState) => ({
          ...oldState,
          error: error as Error,
        }));
      },
    );
  };

  const handleModeChange = (
    _: React.ChangeEvent<unknown>,
    mode: CustomMsgsPidGainConst.LOOP_POSITION | CustomMsgsPidGainConst.LOOP_VELOCITY,
  ) => {
    setState((oldState) => ({ ...oldState, loopType: mode }));

    handleReset();
  };

  function getAxisEnumName(enumValue: number): string {
    const filteredEnumEntries = Object.entries(CustomMsgsPidGainConst).filter(([key, value]) => {
      return key.startsWith("AXIS") && Number(value) === enumValue;
    });

    if (filteredEnumEntries.length > 0 && filteredEnumEntries[0] != null) {
      const enumName = filteredEnumEntries[0][0];
      return enumName.split("_").pop()!;
    } else {
      console.error(`Could not find axis enum name for value ${enumValue}`);
      return "Unknown";
    }
  }

  const handleFocus = (axis: number, gainType: number) => {
    setState((prevState) => ({
      ...prevState,
      editedValues: {
        ...prevState.editedValues,
        [axis]: {
          ...prevState.editedValues[axis],
          [gainType]: pid[state.loopType]![axis]![gainType]!,
        },
      },
    }));
  };

  const handleInputChange = (event: React.ChangeEvent<HTMLInputElement>, axis: number, gainType: number) => {
    const isFocused = Number(gainType) in (state.editedValues[axis] ?? {});
    if (isFocused) {
      setState((prevState) => ({
        ...prevState,
        editedValues: {
          ...prevState.editedValues,
          [axis]: {
            ...prevState.editedValues[axis],
            [gainType]: Number(event.target.value),
          },
        },
      }));
    }
  };

  const handleReset = () => {
    setState((prevState) => ({
      ...prevState,
      editedFields: {},
      editedValues: {},
    }));
  };

  const handleSubmit = () => {
    const request: CustomMsgsSetPidGainsRequest = {
      pid_gains: [],
    };

    Object.entries(state.editedValues).forEach(([axis, gains]) => {
      Object.entries(gains).forEach(([gain, value]) => {
        request.pid_gains.push({
          loop: state.loopType,
          axis: Number(axis),
          gain: Number(gain),
          value: Number(value),
        });
      });
    });

    callService(SET_PID_SERVICE, request);

    handleReset();
  };

  const theme = useTheme();
  return (
    <ThemeProvider theme={theme}>
      <Box m={1}>
        {/* Error messages */}
        {(context.callService == undefined || state.error != undefined) && (
          <Box mb={1}>
            {context.callService == undefined && (
              <Alert variant="filled" severity="error">
                Calling services is not supported by this connection
              </Alert>
            )}
            {state.error != undefined && (
              <Alert variant="filled" severity="error">
                {state.error.message}
              </Alert>
            )}
          </Box>
        )}

        {/* PID Loop Tabs */}
        <Tabs value={state.loopType} onChange={handleModeChange} variant="fullWidth">
          <Tab label="Position" value={CustomMsgsPidGainConst.LOOP_POSITION} />
          <Tab label="Velocity" value={CustomMsgsPidGainConst.LOOP_VELOCITY} />
        </Tabs>

        <TableContainer>
          <Table size="small">
            <TableHead>
              <TableRow>
                <TableCell />
                <TableCell align="center">KP</TableCell>
                <TableCell align="center">KI</TableCell>
                <TableCell align="center">KD</TableCell>
                <TableCell align="center">FF</TableCell>
              </TableRow>
            </TableHead>
            <TableBody>
              {/* Looping through All Axis */}
              {pid[state.loopType] &&
                Object.entries(pid[state.loopType] ?? {}).map(([axis, g]) => (
                  <TableRow key={axis}>
                    <TableCell style={{ whiteSpace: "nowrap", width: "1%" }}>{getAxisEnumName(Number(axis))}</TableCell>
                    {/* Looping through All Gains */}
                    {Object.values(g).map((gain, gainType) => {
                      const isFocused = Number(gainType) in (state.editedValues[Number(axis)] ?? {});
                      return (
                        <TableCell
                          key={gainType}
                          sx={{
                            padding: "1px",
                          }}
                        >
                          <TextField
                            type="text"
                            fullWidth
                            sx={{
                              width: "100%",
                              "& input": {
                                boxSizing: "border-box",
                                padding: "5px",
                              },
                              "& .MuiOutlinedInput-root": {
                                "& fieldset": {
                                  borderColor: `${isFocused ? theme.palette.error.main : theme.palette.divider}`,
                                },
                              },
                            }}
                            value={isFocused ? undefined : gain}
                            onFocus={() => {
                              handleFocus(Number(axis), gainType);
                            }}
                            onChange={(event: React.ChangeEvent<HTMLInputElement>) => {
                              handleInputChange(event, Number(axis), gainType);
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
            <Button variant="contained" fullWidth onClick={handleReset} color="error">
              Reset
            </Button>
          </Grid>
          <Grid item xs={6}>
            <Button variant="contained" fullWidth onClick={handleSubmit} color="success">
              Submit
            </Button>
          </Grid>
        </Grid>
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
