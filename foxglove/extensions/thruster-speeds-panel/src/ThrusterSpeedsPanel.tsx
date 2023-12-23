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

// Enum type for panel modes
enum PanelMode {
  SUBSCRIBING,
  PUBLISHING,
}

type ThrusterSpeedsPanelState = {
  error?: Error | undefined;
  colorScheme?: RenderState["colorScheme"];
  hasError: boolean;
  panelMode: PanelMode;
  repeatPublish: NodeJS.Timeout | null;
  publisherThrusterSpeeds: ThrusterSpeeds;
  subscriberThrusterSpeeds: ThrusterSpeeds;
  tempThrusterSpeeds: ThrusterSpeeds;
};

// Type representing the set of 8 thruster speeds.
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

// Default thruster speeds
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

// Array of thruster keys and thruster order, used to build the thruster grid in the Foxglove extension
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
// Array of thruster keys in the order defined in the robot config file, used to map each thruster to its correct
// position in the speeds array in the "custom_msgs/ThrusterSpeeds" message
const thrustersInOrder: (keyof ThrusterSpeeds)[] = allThrusterOrders[ROBOT] as (keyof ThrusterSpeeds)[];

// React component for Thruster Speeds Panel
function ThrusterSpeedsPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  // Panel state initialization
  const [renderDone, setRenderDone] = useState<() => void | undefined>();
  const firstMount = useRef(true);
  const theme = useTheme();
  const [state, setState] = useState<ThrusterSpeedsPanelState>({
    // In the PUBLISHING mode, denotes whether all entered values in the panel are valid.
    // If false, display an error warning which prevents the panel from publishing.
    hasError: false,
    panelMode: PanelMode.SUBSCRIBING,
    // If publishing, holds the NodeJS.Timeout object used to publish messages at a constant rate, otherwise undefined
    repeatPublish: null,
    // Thruster speeds to be published
    publisherThrusterSpeeds: { ...defaultThrusterSpeeds },
    // Thruster speeds subscribed from the message
    subscriberThrusterSpeeds: { ...defaultThrusterSpeeds },
    // Holds temporary values of thruster speeds that the user entered before publishing
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

  // useEffect hook for subscribing to THRUSTER_SPEEDS_TOPIC
  useEffect(() => {
    context.saveState({ topic: THRUSTER_SPEEDS_TOPIC });
    context.subscribe([{ topic: THRUSTER_SPEEDS_TOPIC }]);
  }, [context]);

  // useEffect hook to start or stop publishing messages at a constant rate.
  // Returns early when the panel is first mounted, which is needed for correct panel behavior subsequently.
  useEffect(() => {
    if (firstMount.current) {
      firstMount.current = false;
      return;
    }

    toggleInterval();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [state.publisherThrusterSpeeds]);

  // useEffect hook for rendering and watching renderState. Saves the values from the most recent message.
  useEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setRenderDone(() => done);
      setState((oldState) => ({ ...oldState, colorScheme: renderState.colorScheme }));

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
  }, [context]);

  // Callback function to publish thruster speeds
  const publishSpeeds = useCallback(() => {
    // Message creation
    const message = {
      header: {
        seq: 0,
        stamp: {
          secs: 0,
          nsecs: 0,
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

    // Publishes the message to THRUSTER_SPEEDS_TOPIC
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
  }, [context, state.publisherThrusterSpeeds]);

  // Function to validate individual input values for each thruster
  const validateInput = (value: number | "") => {
    return (
      value === "" || (Number.isInteger(Number(value)) && value >= MIN_THRUSTER_SPEED && value <= MAX_THRUSTER_SPEED)
    );
  };

  // Event handler for mode change between SUBSCRIBING and PUBLISHING
  const handleModeChange = (_: React.SyntheticEvent, mode: PanelMode) => {
    setState((oldState) => ({ ...oldState, panelMode: mode }));
  };

  // Function to toggle interval for publishing
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

  // Function to generate publisher speeds from tempThrusterSpeeds. If any value of tempThrusterSpeeds is "",
  // the the latest subscribed speed for that thruster is used instead.
  const generatePublisherSpeeds = () => {
    const newThrusterSpeeds: ThrusterSpeeds = { ...state.tempThrusterSpeeds };
    thrusters.forEach((thruster: keyof ThrusterSpeeds) => {
      if (state.tempThrusterSpeeds[thruster] === "") {
        newThrusterSpeeds[thruster] = state.subscriberThrusterSpeeds[thruster];
      }
    });

    return newThrusterSpeeds;
  };

  // Event handler for updating tempThrusterSpeeds upon user input
  const updateTempSpeeds = (event: React.ChangeEvent<HTMLInputElement>) => {
    let hasError = false;
    const value = event.target.value;

    // Loops through and checks whether all user input values are valid
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
        {/* SUBSCRIBING and PUBLISHING tabs */}
        <Tabs value={state.panelMode} onChange={handleModeChange} variant="fullWidth">
          <Tab label="Subscribing" value={PanelMode.SUBSCRIBING} />
          <Tab label="Publishing" value={PanelMode.PUBLISHING} />
        </Tabs>
        {/* Alert to be displayed if the panel is in PUBLISHING mode but cannot publish to THRUSTER_SPEEDS_TOPIC */}
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
          {/* Grid for displaying thruster speeds. If in SUBSCRIBING mode, displays subscribed speeds, otherwise
          displays TextFields for user to input thruster speeds values */}
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
                  fullWidth
                />
              </Grid>
            ))}
          </Grid>
        </div>
        <div style={{ display: "flex", justifyContent: "center", padding: "5px" }}>
          {state.panelMode === PanelMode.SUBSCRIBING ? (
            <></>
          ) : state.hasError ? (
            // Alert to be displayed if any user input thruster speeds are invalid
            <Alert variant="filled" severity="error">
              The speed value for each thruster must be an integer between -128 and 127!
            </Alert>
          ) : (
            // Button to start and stop publishing
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
