import { ros1 } from "@foxglove/rosmsg-msgs-common";
import { PanelExtensionContext, RenderState, Immutable } from "@foxglove/studio";
import CheckCircleOutlineIcon from "@mui/icons-material/CheckCircleOutline";
import { TextField, Button, Alert } from "@mui/material";
import Grid from "@mui/material/Unstable_Grid2";
import { useCallback, useEffect, useLayoutEffect, useState } from "react";
import * as React from "react";
import { createRoot } from "react-dom/client";

type ThrusterSpeeds = {
  frontLeft: number;
  frontRight: number;
  backLeft: number;
  backRight: number;
  bottomFrontLeft: number;
  bottomFrontRight: number;
  bottomBackLeft: number;
  bottomBackRight: number;
};

type State = {
  topicName: string;
  request: string;
  error?: Error | undefined;
  colorScheme?: RenderState["colorScheme"];
  thrusterSpeeds: ThrusterSpeeds;
  tempThrusterSpeeds: ThrusterSpeeds;
  hasError: boolean;
};

function ThrusterSpeedsPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<() => void | undefined>();
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

  const [state, setState] = useState<State>({
    topicName: "",
    request: "{}",
    hasError: false,
    thrusterSpeeds: {
      frontLeft: 0,
      frontRight: 0,
      backLeft: 0,
      backRight: 0,
      bottomFrontLeft: 0,
      bottomFrontRight: 0,
      bottomBackLeft: 0,
      bottomBackRight: 0,
    },
    tempThrusterSpeeds: {
      frontLeft: 0,
      frontRight: 0,
      backLeft: 0,
      backRight: 0,
      bottomFrontLeft: 0,
      bottomFrontRight: 0,
      bottomBackLeft: 0,
      bottomBackRight: 0,
    },
  });

  useLayoutEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setState((oldState) => ({ ...oldState, colorScheme: renderState.colorScheme }));
      setRenderDone(() => done);
    };
  }, [context]);

  context.watch("colorScheme");

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // useEffect(() => {
  //   publishSpeeds();
  // }, []);

  useEffect(() => {
    const repeatPublish = setInterval(() => {
      publishSpeeds();
    }, publishRate);

    return () => {
      clearInterval(repeatPublish);
    };
  }, [state.thrusterSpeeds]);

  const publishSpeeds = useCallback(() => {
    const topicName = "offboard/thruster_speeds";
    const messageType = "custom_msgs/ThrusterSpeeds";
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

    const msgDefinition = {
      name: "custom_msgs/ThrusterSpeeds",
      definitions: [
        {
          isArray: false,
          isComplex: true,
          name: "header",
          type: "std_msgs/Header",
        },
        {
          isArray: true,
          arrayLength: 8,
          isComplex: false,
          name: "speeds",
          type: "int8",
        },
      ],
    };

    const message = {
      header: {
        seq: 0,
        stamp: {
          secs: 0,
          nsecs: 0,
        },
        frame_id: "",
      },
      speeds: thrustersInOrder.map((thruster: keyof ThrusterSpeeds) => state.thrusterSpeeds[thruster]),
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
          ["custom_msgs/ThrusterSpeeds", msgDefinition],
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
  }, [context, state.thrusterSpeeds]);

  const validateInput = (number: number) => {
    return number >= -128 && number <= 127;
  };

  const updateSpeeds = (event: React.ChangeEvent<HTMLInputElement>) => {
    let hasError = false;
    const value = event.target.value === "" ? "0" : event.target.value;

    if (value !== "" && (Number.isNaN(Number(value)) || parseInt(value) !== parseFloat(value))) {
      hasError = true;
    } else {
      thrusters.forEach((thruster: keyof ThrusterSpeeds) => {
        const speed: number = thruster === event.target.id ? parseInt(value) : state.tempThrusterSpeeds[thruster];
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
    <div style={{ padding: "6px" }}>
      <div style={{ padding: "6px" }}>
        {(context.advertise == undefined || context.publish == undefined) && (
          <Alert variant="filled" severity="error">
            Publishing topics is not supported by this connection
          </Alert>
        )}
      </div>
      <div>
        <Grid container rowSpacing={1} columnSpacing={{ xs: 1, sm: 2, md: 3 }}>
          {thrusters.map((thruster) => (
            <Grid key={thruster} xs={6}>
              <TextField
                key={thruster}
                id={thruster}
                error={!validateInput(state.tempThrusterSpeeds[thruster])}
                label={thruster}
                size="small"
                variant="outlined"
                defaultValue={0}
                onChange={updateSpeeds}
              />
            </Grid>
          ))}
        </Grid>
      </div>
      <div style={{ display: "flex", justifyContent: "center", padding: "5px" }}>
        {state.hasError ? (
          <Alert variant="filled" severity="error">
            The speed value for each thruster must be an integer between -128 and 127
          </Alert>
        ) : (
          <Button
            variant="contained"
            color="success"
            endIcon={<CheckCircleOutlineIcon />}
            onClick={() => {
              setState((oldState) => ({
                ...oldState,
                thrusterSpeeds: { ...state.tempThrusterSpeeds },
              }));
            }}
            disabled={context.callService == undefined}
          >
            Publish
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
