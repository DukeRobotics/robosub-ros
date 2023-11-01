import { ros1 } from "@foxglove/rosmsg-msgs-common";
import { PanelExtensionContext, RenderState, Immutable, MessageEvent } from "@foxglove/studio";
import { CheckCircleOutline, HighlightOff } from "@mui/icons-material";
import { TextField, Button, Alert, Tab, Tabs } from "@mui/material";
import Grid from "@mui/material/Unstable_Grid2";
import { useCallback, useEffect, useLayoutEffect, useState, useRef } from "react";
import * as React from "react";
import { createRoot } from "react-dom/client";

import { custom_msgs } from "../../../msgdefs/dist";

enum PanelMode {
  SUBSCRIBING,
  PUBLISHING,
}

type State = {
  error?: Error | undefined;
  colorScheme?: RenderState["colorScheme"];
  hasError: boolean;
  panelMode: PanelMode;
  repeatPublish: NodeJS.Timeout | null;
  publisherThrusterSpeeds: ThrusterSpeeds;
  subscriberThrusterSpeeds: ThrusterSpeeds;
  tempThrusterSpeeds: ThrusterSpeeds;
};

type ThrusterSpeeds = {
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

const topicName = "/offboard/thruster_speeds";
const messageType = "custom_msgs/ThrusterSpeeds";
const publishRate = 100;

const thrustersInOrder: (keyof ThrusterSpeeds)[] = [
  "bottomFrontLeft",
  "frontLeft",
  "frontRight",
  "bottomFrontRight",
  "bottomBackLeft",
  "backLeft",
  "bottomBackRight",
  "backRight",
];
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

function ThrusterSpeedsPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<() => void | undefined>();
  const firstMount = useRef(true);
  const [state, setState] = useState<State>({
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
    context.saveState({ topic: topicName });
    context.subscribe([{ topic: topicName }]);
  }, [context]);

  useEffect(() => {
    if (firstMount.current) {
      firstMount.current = false;
      return;
    }

    toggleInterval();
  }, [state.publisherThrusterSpeeds]);

  useLayoutEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setRenderDone(() => done);
      setState((oldState) => ({ ...oldState, colorScheme: renderState.colorScheme }));

      // Save the most recent message on our topic.
      if (renderState.currentFrame && renderState.currentFrame.length > 0) {
        const latestFrame = renderState.currentFrame[renderState.currentFrame.length - 1] as MessageEvent<any>;
        let newSpeeds: ThrusterSpeeds = { ...defaultThrusterSpeeds };
        thrustersInOrder.forEach((thruster: keyof ThrusterSpeeds, index) => {
          newSpeeds = { ...newSpeeds, [thruster]: latestFrame.message?.speeds[index] };
        });
        setState((oldState) => ({ ...oldState, subscriberThrusterSpeeds: newSpeeds }));
      }
    };

    context.watch("currentFrame");
    context.watch("colorScheme");
  }, [context]);

  const publishSpeeds = useCallback(() => {
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

    try {
      context.advertise(`/${topicName}`, messageType, {
        datatypes: new Map([
          ["std_msgs/Header", ros1["std_msgs/Header"]],
          ["std_msgs/Int8", ros1["std_msgs/Int8"]],
          ["custom_msgs/ThrusterSpeeds", custom_msgs["custom_msgs/ThrusterSpeeds"]],
        ]),
      });
      context.publish(`/${topicName}`, message);

      setState((oldState) => ({
        ...oldState,
        error: undefined,
      }));
    } catch (error) {
      setState((oldState) => ({ ...oldState, error: error as Error }));
      console.error(error);
    }
  }, [context, state.publisherThrusterSpeeds]);

  const validateInput = (value: number | "") => {
    return value === "" || (value >= -128 && value <= 127);
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
                error={
                  state.panelMode === PanelMode.SUBSCRIBING ? false : !validateInput(state.tempThrusterSpeeds[thruster])
                }
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
