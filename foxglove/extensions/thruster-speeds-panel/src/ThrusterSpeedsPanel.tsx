import { allDatatypeMaps } from "@duke-robotics/defs/datatype_maps";
import { CustomMsgsThrusterSpeeds } from "@duke-robotics/defs/types";
import useTheme from "@duke-robotics/theme";
import { PanelExtensionContext, RenderState, Immutable, MessageEvent } from "@foxglove/studio";
import { CheckCircleOutline, HighlightOff } from "@mui/icons-material";
import { TextField, Button, Alert, Tab, Tabs, CssBaseline } from "@mui/material";
import Grid from "@mui/material/Unstable_Grid2";
import { ThemeProvider } from "@mui/material/styles";
import { useCallback, useEffect, useState, useRef } from "react";
import { createRoot } from "react-dom/client";

import { allThrusterOrders } from "../dist";

enum PanelMode {
  SUBSCRIBING,
  PUBLISHING,
}

type ThrusterSpeedsPanelState = {
  error?: Error | undefined;
  colorScheme?: RenderState["colorScheme"];
  currentTime?: RenderState["currentTime"];
  hasError: boolean;
  panelMode: PanelMode;
  repeatPublish: NodeJS.Timeout | null;
  publisherThrusterSpeeds: ThrusterSpeeds;
  subscriberThrusterSpeeds: ThrusterSpeeds;
  tempThrusterSpeeds: ThrusterSpeeds;
};

export type ThrusterSpeeds = {
  frontLeft: number | "";
  frontRight: number | "";
  backLeft: number | "";
  backRight: number | "";
  bottomFrontLeft: number | "";
  bottomFrontRight: number | "";
  bottomBackLeft: number | "";
  bottomBackRight: number | "";
};

const defaultThrusterSpeeds: ThrusterSpeeds = {
  frontLeft: 0,
  frontRight: 0,
  backLeft: 0,
  backRight: 0,
  bottomFrontLeft: 0,
  bottomFrontRight: 0,
  bottomBackLeft: 0,
  bottomBackRight: 0,
};

const ROBOT = "OOGWAY";
const MIN_THRUSTER_SPEED = -128;
const MAX_THRUSTER_SPEED = 127;
const THRUSTER_SPEEDS_TOPIC = "/offboard/thruster_speeds";
const THRUSTER_SPEEDS_MESSAGE_TYPE = "custom_msgs/ThrusterSpeeds";

// This is the delay, in miliseconds, between two consecutive messages published by the panel.
// A 100ms delay means the panel is publishing messages at 10Hz.
const publishRate = 100;

const thrusters: (keyof ThrusterSpeeds)[] = [
  "frontLeft",
  "frontRight",
  "backLeft",
  "backRight",
  "bottomFrontLeft",
  "bottomFrontRight",
  "bottomBackLeft",
  "bottomBackRight",
];
const thrustersInOrder: (keyof ThrusterSpeeds)[] = allThrusterOrders[ROBOT] as (keyof ThrusterSpeeds)[];

function ThrusterSpeedsPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<() => void | undefined>();
  const firstMount = useRef(true);
  const theme = useTheme();
  const [state, setState] = useState<ThrusterSpeedsPanelState>({
    hasError: false,
    panelMode: PanelMode.SUBSCRIBING,
    repeatPublish: null,
    publisherThrusterSpeeds: { ...defaultThrusterSpeeds },
    subscriberThrusterSpeeds: { ...defaultThrusterSpeeds },
    tempThrusterSpeeds: {
      frontLeft: "",
      frontRight: "",
      backLeft: "",
      backRight: "",
      bottomFrontLeft: "",
      bottomFrontRight: "",
      bottomBackLeft: "",
      bottomBackRight: "",
    },
  });

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  useEffect(() => {
    context.saveState({ topic: THRUSTER_SPEEDS_TOPIC });
    context.subscribe([{ topic: THRUSTER_SPEEDS_TOPIC }]);
  }, [context]);

  useEffect(() => {
    if (firstMount.current) {
      firstMount.current = false;
      return;
    }

    toggleInterval();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [state.publisherThrusterSpeeds]);

  useEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setRenderDone(() => done);
      setState((oldState) => ({ ...oldState, colorScheme: renderState.colorScheme }));

      if (renderState.currentTime != undefined) {
        console.log(renderState.currentTime);
        setState((oldState) => ({ ...oldState, currentTime: renderState.currentTime }));
      }

      // Save the most recent message on our topic.
      if (renderState.currentFrame && renderState.currentFrame.length > 0) {
        const latestFrame = renderState.currentFrame[
          renderState.currentFrame.length - 1
        ] as MessageEvent<CustomMsgsThrusterSpeeds>;
        const newSpeeds: ThrusterSpeeds = { ...defaultThrusterSpeeds };
        thrustersInOrder.forEach((thruster: keyof ThrusterSpeeds, index) => {
          newSpeeds[thruster] = latestFrame.message.speeds[index] as number | "";
        });
        setState((oldState) => ({ ...oldState, subscriberThrusterSpeeds: newSpeeds }));
      }
    };

    context.watch("currentFrame");
    context.watch("colorScheme");
    context.watch("currentTime");
  }, [context]);

  const publishSpeeds = useCallback(() => {
    console.log(state.currentTime);
    const message = {
      header: {
        seq: 0,
        stamp: {
          secs: state.currentTime?.sec ?? 0,
          nsecs: state.currentTime?.nsec ?? 0,
        },
        frame_id: "",
      },
      speeds: thrustersInOrder.map((thruster: keyof ThrusterSpeeds) => state.publisherThrusterSpeeds[thruster]),
    };

    if (!context.advertise) {
      return;
    }
    if (!context.publish) {
      return;
    }

    try {
      context.advertise(`/${THRUSTER_SPEEDS_TOPIC}`, THRUSTER_SPEEDS_MESSAGE_TYPE, {
        datatypes: allDatatypeMaps["custom_msgs"][THRUSTER_SPEEDS_MESSAGE_TYPE],
      });
      context.publish(`/${THRUSTER_SPEEDS_TOPIC}`, message);

      setState((oldState) => ({
        ...oldState,
        error: undefined,
      }));
    } catch (error) {
      setState((oldState) => ({ ...oldState, error: error as Error }));
      console.error(error);
    }
  }, [context, state.publisherThrusterSpeeds, state.currentTime]);

  const validateInput = (value: number | "") => {
    return (
      value === "" || (Number.isInteger(Number(value)) && value >= MIN_THRUSTER_SPEED && value <= MAX_THRUSTER_SPEED)
    );
  };

  const handleModeChange = (_: React.SyntheticEvent, mode: PanelMode) => {
    setState((oldState) => ({ ...oldState, panelMode: mode }));
  };

  const toggleInterval = () => {
    if (state.repeatPublish == null) {
      setState((oldState) => ({
        ...oldState,
        repeatPublish: setInterval(() => {
          publishSpeeds();
        }, publishRate),
      }));
    } else {
      clearInterval(state.repeatPublish);
      setState((oldState) => ({
        ...oldState,
        repeatPublish: null,
      }));
    }
  };

  const generatePublisherSpeeds = () => {
    const newThrusterSpeeds: ThrusterSpeeds = { ...state.tempThrusterSpeeds };
    thrusters.forEach((thruster: keyof ThrusterSpeeds) => {
      if (state.tempThrusterSpeeds[thruster] === "") {
        newThrusterSpeeds[thruster] = state.subscriberThrusterSpeeds[thruster];
      }
    });

    return newThrusterSpeeds;
  };

  const updateTempSpeeds = (event: React.ChangeEvent<HTMLInputElement>) => {
    let hasError = false;
    const value = event.target.value;

    if (value !== "" && (Number.isNaN(Number(value)) || parseInt(value) !== parseFloat(value))) {
      hasError = true;
    } else {
      thrusters.forEach((thruster: keyof ThrusterSpeeds) => {
        const speed: number | "" =
          thruster !== event.target.id ? state.tempThrusterSpeeds[thruster] : value !== "" ? parseInt(value) : "";
        if (!validateInput(speed)) {
          hasError = true;
        }
      });
    }

    setState((oldState) => ({
      ...oldState,
      hasError,
      tempThrusterSpeeds: {
        ...state.tempThrusterSpeeds,
        [event.target.id]: value,
      },
    }));
  };

  return (
    <ThemeProvider theme={theme}>
      <CssBaseline />
      <div style={{ padding: "5px" }}>
        <Tabs value={state.panelMode} onChange={handleModeChange} variant="fullWidth">
          <Tab label="Subscribing" value={PanelMode.SUBSCRIBING} />
          <Tab label="Publishing" value={PanelMode.PUBLISHING} />
        </Tabs>
        <div style={{ padding: "5px" }}>
          {state.panelMode === PanelMode.SUBSCRIBING ? (
            <></>
          ) : (
            (context.advertise == undefined || context.publish == undefined) && (
              <Alert variant="filled" severity="error">
                Publishing topics is not supported by this connection
              </Alert>
            )
          )}
        </div>
        <div>
          <Grid container rowSpacing={1} columnSpacing={0}>
            {thrusters.map((thruster) => (
              <Grid key={thruster} xs={6}>
                <TextField
                  key={thruster}
                  id={thruster}
                  error={state.panelMode === PanelMode.PUBLISHING && !validateInput(state.tempThrusterSpeeds[thruster])}
                  label={thruster}
                  size="small"
                  variant={state.panelMode === PanelMode.SUBSCRIBING ? "filled" : "outlined"}
                  value={
                    state.panelMode === PanelMode.SUBSCRIBING
                      ? state.subscriberThrusterSpeeds[thruster]
                      : state.tempThrusterSpeeds[thruster]
                  }
                  InputProps={state.panelMode === PanelMode.SUBSCRIBING ? { readOnly: true } : {}}
                  defaultValue={state.panelMode === PanelMode.SUBSCRIBING ? false : 0}
                  onChange={updateTempSpeeds}
                />
              </Grid>
            ))}
          </Grid>
        </div>
        <div style={{ display: "flex", justifyContent: "center", padding: "5px" }}>
          {state.panelMode === PanelMode.SUBSCRIBING ? (
            <></>
          ) : state.hasError ? (
            <Alert variant="filled" severity="error">
              The speed value for each thruster must be an integer between -128 and 127!
            </Alert>
          ) : (
            <Button
              variant="contained"
              color={state.repeatPublish == null ? "success" : "error"}
              endIcon={state.repeatPublish == null ? <CheckCircleOutline /> : <HighlightOff />}
              onClick={
                state.repeatPublish == null
                  ? () => {
                      setState((oldState) => ({
                        ...oldState,
                        publisherThrusterSpeeds: generatePublisherSpeeds(),
                      }));
                    }
                  : toggleInterval
              }
              disabled={context.callService == undefined}
            >
              {state.repeatPublish == null ? "Start Publishing" : "Stop Publishing"}
            </Button>
          )}
        </div>
      </div>
    </ThemeProvider>
  );
}

export function initThrusterSpeedsPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<ThrusterSpeedsPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
