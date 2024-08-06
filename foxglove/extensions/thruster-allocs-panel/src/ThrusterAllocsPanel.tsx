import { allDatatypeMaps } from "@duke-robotics/defs/datatype_maps";
import { CustomMsgsThrusterAllocs } from "@duke-robotics/defs/types";
import useTheme from "@duke-robotics/theme";
import { PanelExtensionContext, RenderState, Immutable, MessageEvent } from "@foxglove/extension";
import { TextField, Button, Alert, Tab, Tabs, CssBaseline, Box } from "@mui/material";
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

type ThrusterAllocsPanelState = {
  error?: Error | undefined;
  colorScheme?: RenderState["colorScheme"];
  hasError: boolean;
  panelMode: PanelMode;
  repeatPublish: NodeJS.Timeout | null;
  publisherThrusterAllocs: ThrusterAllocs;
  subscriberThrusterAllocs: ThrusterAllocs;
  tempThrusterAllocs: ThrusterAllocs;
};

// Type representing the set of 8 thruster allocs.
export type ThrusterAllocs = {
  topFrontLeft: number | "";
  topFrontRight: number | "";
  topBackLeft: number | "";
  topBackRight: number | "";
  bottomFrontLeft: number | "";
  bottomFrontRight: number | "";
  bottomBackLeft: number | "";
  bottomBackRight: number | "";
};

// Default thruster allocs
const defaultThrusterAllocs: ThrusterAllocs = {
  topFrontLeft: 0,
  topFrontRight: 0,
  topBackLeft: 0,
  topBackRight: 0,
  bottomFrontLeft: 0,
  bottomFrontRight: 0,
  bottomBackLeft: 0,
  bottomBackRight: 0,
};

const ROBOT = "OOGWAY";
const MIN_THRUSTER_ALLOC = -1;
const MAX_THRUSTER_ALLOC = 1;
const THRUSTER_ALLOCS_TOPIC = "/controls/thruster_allocs";
const THRUSTER_ALLOCS_MESSAGE_TYPE = "custom_msgs/ThrusterAllocs";

// This is the delay, in miliseconds, between two consecutive messages published by the panel.
// A 100ms delay means the panel is publishing messages at 10Hz.
const publishRate = 100;

// Array of thruster keys and thruster order, used to build the thruster grid in the Foxglove extension
const thrusters: (keyof ThrusterAllocs)[] = [
  "topFrontLeft",
  "topFrontRight",
  "topBackLeft",
  "topBackRight",
  "bottomFrontLeft",
  "bottomFrontRight",
  "bottomBackLeft",
  "bottomBackRight",
];
// Array of thruster keys in the order defined in the robot config file, used to map each thruster to its correct
// position in the allocs array in the "custom_msgs/ThrusterAllocs" message
const thrustersInOrder: (keyof ThrusterAllocs)[] = allThrusterOrders[ROBOT] as (keyof ThrusterAllocs)[];

// React component for Thruster Allocs Panel
function ThrusterAllocsPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  // Panel state initialization
  const [renderDone, setRenderDone] = useState<() => void | undefined>();
  const firstMount = useRef(true);
  const theme = useTheme();
  const [state, setState] = useState<ThrusterAllocsPanelState>(() => {
    const initialState = context.initialState as ThrusterAllocsPanelState | undefined;

    return {
      // In the PUBLISHING mode, denotes whether all entered values in the panel are valid.
      // If false, display an error warning which prevents the panel from publishing.
      hasError: initialState?.hasError ?? false,
      panelMode: initialState?.panelMode ?? PanelMode.SUBSCRIBING,
      // If publishing, holds the NodeJS.Timeout object used to publish messages at a constant rate, otherwise undefined
      repeatPublish: null,
      // Thruster allocs to be published
      publisherThrusterAllocs: { ...defaultThrusterAllocs },
      // Thruster allocs subscribed from the message
      subscriberThrusterAllocs: { ...defaultThrusterAllocs },
      // Holds temporary values of thruster allocs that the user entered before publishing
      tempThrusterAllocs: initialState?.tempThrusterAllocs ?? {
        topFrontLeft: "",
        topFrontRight: "",
        topBackLeft: "",
        topBackRight: "",
        bottomFrontLeft: "",
        bottomFrontRight: "",
        bottomBackLeft: "",
        bottomBackRight: "",
      },
    };
  });

  // Save state upon change
  useEffect(() => {
    context.saveState(state);
  }, [state, context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // useEffect hook for subscribing to THRUSTER_ALLOCS_TOPIC
  useEffect(() => {
    context.saveState({ topic: THRUSTER_ALLOCS_TOPIC });
    context.subscribe([{ topic: THRUSTER_ALLOCS_TOPIC }]);
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
  }, [state.publisherThrusterAllocs]);

  // useEffect hook for rendering and watching renderState. Saves the values from the most recent message.
  useEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setRenderDone(() => done);
      setState((oldState) => ({ ...oldState, colorScheme: renderState.colorScheme }));

      // Save the most recent message on our topic.
      if (renderState.currentFrame && renderState.currentFrame.length > 0) {
        const latestFrame = renderState.currentFrame[
          renderState.currentFrame.length - 1
        ] as MessageEvent<CustomMsgsThrusterAllocs>;
        const newAllocs: ThrusterAllocs = { ...defaultThrusterAllocs };
        thrustersInOrder.forEach((thruster: keyof ThrusterAllocs, index) => {
          newAllocs[thruster] = latestFrame.message.allocs[index] as number | "";
        });
        setState((oldState) => ({ ...oldState, subscriberThrusterAllocs: newAllocs }));
      }
    };

    context.watch("currentFrame");
    context.watch("colorScheme");
  }, [context]);

  // Callback function to publish thruster allocs
  const publishAllocs = useCallback(() => {
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
      allocs: thrustersInOrder.map((thruster: keyof ThrusterAllocs) => state.publisherThrusterAllocs[thruster]),
    };

    if (!context.advertise) {
      return;
    }
    if (!context.publish) {
      return;
    }

    // Publishes the message to THRUSTER_ALLOCS_TOPIC
    try {
      context.advertise(THRUSTER_ALLOCS_TOPIC, THRUSTER_ALLOCS_MESSAGE_TYPE, {
        datatypes: allDatatypeMaps["custom_msgs"][THRUSTER_ALLOCS_MESSAGE_TYPE],
      });
      context.publish(THRUSTER_ALLOCS_TOPIC, message);

      setState((oldState) => ({
        ...oldState,
        error: undefined,
      }));
    } catch (error) {
      setState((oldState) => ({ ...oldState, error: error as Error }));
      console.error(error);
    }
  }, [context, state.publisherThrusterAllocs]);

  // Function to validate individual input values for each thruster
  const validateInput = (value: number | "") => {
    return value === "" || (value >= MIN_THRUSTER_ALLOC && value <= MAX_THRUSTER_ALLOC);
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
          publishAllocs();
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

  // Function to generate publisher allocs from tempThrusterAllocs. If any value of tempThrusterAllocs is "",
  // then the latest subscribed alloc for that thruster is used instead.
  const generatePublisherAllocs = () => {
    const newThrusterAllocs: ThrusterAllocs = { ...state.tempThrusterAllocs };
    thrusters.forEach((thruster: keyof ThrusterAllocs) => {
      if (state.tempThrusterAllocs[thruster] === "") {
        newThrusterAllocs[thruster] = state.subscriberThrusterAllocs[thruster];
      }
    });

    return newThrusterAllocs;
  };

  // Event handler for updating tempThrusterAllocs upon user input
  const updateTempAllocs = (event: React.ChangeEvent<HTMLInputElement>) => {
    let hasError = false;
    const value = event.target.value;

    // Loops through and checks whether all user input values are valid
    if (value !== "" && Number.isNaN(Number(value))) {
      hasError = true;
    } else {
      thrusters.forEach((thruster: keyof ThrusterAllocs) => {
        const alloc: number | "" =
          thruster !== event.target.id ? state.tempThrusterAllocs[thruster] : value !== "" ? Number(value) : "";
        if (!validateInput(alloc)) {
          hasError = true;
        }
      });
    }

    setState((oldState) => ({
      ...oldState,
      hasError,
      tempThrusterAllocs: {
        ...state.tempThrusterAllocs,
        [event.target.id]: value,
      },
    }));
  };

  return (
    <ThemeProvider theme={theme}>
      <CssBaseline />
      <Box m={1}>
        {/* SUBSCRIBING and PUBLISHING tabs */}
        <Tabs value={state.panelMode} onChange={handleModeChange} variant="fullWidth">
          <Tab label="Subscribing" value={PanelMode.SUBSCRIBING} />
          <Tab label="Publishing" value={PanelMode.PUBLISHING} />
        </Tabs>
        {/* Alert to be displayed if the panel is in PUBLISHING mode but cannot publish to THRUSTER_ALLOCS_TOPIC */}
        <Box my={1}>
          {state.panelMode === PanelMode.SUBSCRIBING ? (
            <></>
          ) : (
            (context.advertise == undefined || context.publish == undefined) && (
              <Alert variant="filled" severity="error">
                Publishing topics is not supported by this connection.
              </Alert>
            )
          )}
        </Box>
        <Box>
          {/* Grid for displaying thruster allocs. If in SUBSCRIBING mode, displays subscribed allocs, otherwise
          displays TextFields for user to input thruster allocs values */}
          <Grid container rowSpacing={1} columnSpacing={0}>
            {thrusters.map((thruster) => (
              <Grid key={thruster} xs={6}>
                <TextField
                  type="number"
                  key={thruster}
                  id={thruster}
                  error={state.panelMode === PanelMode.PUBLISHING && !validateInput(state.tempThrusterAllocs[thruster])}
                  label={thruster}
                  size="small"
                  variant={state.panelMode === PanelMode.SUBSCRIBING ? "filled" : "outlined"}
                  value={
                    state.panelMode === PanelMode.SUBSCRIBING
                      ? state.subscriberThrusterAllocs[thruster]
                      : state.tempThrusterAllocs[thruster]
                  }
                  inputProps={state.panelMode === PanelMode.PUBLISHING ? { step: 0.1 } : {}}
                  InputProps={state.panelMode === PanelMode.SUBSCRIBING ? { readOnly: true } : {}}
                  defaultValue={state.panelMode === PanelMode.SUBSCRIBING ? false : 0}
                  onChange={updateTempAllocs}
                  fullWidth
                />
              </Grid>
            ))}
          </Grid>
        </Box>
        <Box my={1}>
          {state.panelMode === PanelMode.SUBSCRIBING ? (
            <></>
          ) : state.hasError ? (
            // Alert to be displayed if any user input thruster allocs are invalid
            <Alert variant="filled" severity="error">
              The alloc value for each thruster must be a float between -1 and 1.
            </Alert>
          ) : (
            // Button to start and stop publishing
            <Button
              fullWidth
              variant="contained"
              color={state.repeatPublish == null ? "success" : "error"}
              onClick={
                state.repeatPublish == null
                  ? () => {
                      setState((oldState) => ({
                        ...oldState,
                        publisherThrusterAllocs: generatePublisherAllocs(),
                      }));
                    }
                  : toggleInterval
              }
              disabled={context.callService == undefined}
            >
              {state.repeatPublish == null ? "Start Publishing" : "Stop Publishing"}
            </Button>
          )}
        </Box>
      </Box>
    </ThemeProvider>
  );
}

export function initThrusterAllocsPanel(context: PanelExtensionContext): () => void {
  context.panelElement.style.overflow = "auto"; // Enable scrolling

  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<ThrusterAllocsPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
