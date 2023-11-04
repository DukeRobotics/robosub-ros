import { Immutable, PanelExtensionContext, RenderState } from "@foxglove/studio";
import { TextField, Button, Alert, Tab, Tabs, Grid } from "@mui/material";
import { JsonViewer } from "@textea/json-viewer";
import { useEffect, useLayoutEffect, useState } from "react";
import React = require("react");
import { JSX } from "react/jsx-runtime";
import { createRoot } from "react-dom/client";

type State = {
  serviceName: string;
  request: string;
  response?: unknown;
  error?: Error | undefined;
  colorScheme?: RenderState["colorScheme"];
  panelMode: PanelMode;
};
// Triple nested dictionary to get PID values
const gains: Record<string, number> = {
  GAIN_KP: 0,
  GAIN_KI: 0,
  GAIN_KD: 0,
  GAIN_FF: 0,
};
const axes: Record<string, Record<string, number>> = {
  AXIS_X: { ...gains },
  AXIS_Y: { ...gains },
  AXIS_Z: { ...gains },
  AXIS_ROLL: { ...gains },
  AXIS_PITCH: { ...gains },
  AXIS_YAW: { ...gains },
};
const pid: Record<string, Record<string, Record<string, number>>> = {
  POSITION_PID: { ...axes },
  VELOCITY_PID: { ...axes },
};


const topicName = "/current-pid";

enum PanelMode {
  SUBSCRIBING,
  EDITING,
}

function PIDPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [state, setState] = useState<State>({ serviceName: "", request: "{}", panelMode: PanelMode.SUBSCRIBING });

  // Update color scheme
  useLayoutEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setState((oldState) => ({ ...oldState, colorScheme: renderState.colorScheme }));
      setRenderDone(() => done);
    };
  }, [context]);
  context.watch("colorScheme");
  context.watch("currentTime");

  useEffect(() => {
    context.saveState({ topic: topicName });
    context.subscribe([{ topic: topicName }]);
  }, [context]);

  // Call our done function at the end of each render
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // Call a service with a given request
  const callService = async (serviceName: string, request: string) => {
    if (!context.callService) {
      return;
    }

    try {
      const response = await context.callService(serviceName, JSON.parse(request));
      JSON.stringify(response); // Attempt serializing the response, to throw an error on failure
      setState((oldState) => ({
        ...oldState,
        response,
        error: undefined,
      }));
    } catch (error) {
      setState((oldState) => ({ ...oldState, error: error as Error }));
      console.error(error);
    }
  };

  const handleModeChange = (_: React.SyntheticEvent, mode: PanelMode) => {
    setState((oldState) => ({ ...oldState, panelMode: mode }));
  };

  // Close callService with the current state for use in the button
  const callServiceWithRequest = () => {
    void callService(state.serviceName, state.request);
  };

  return (
    <div style={{ padding: "1rem" }}>
      {context.callService == undefined && (
        <Alert variant="filled" severity="error">
          Calling services is not supported by this connection
        </Alert>
      )}
      <Tabs value={state.panelMode} onChange={handleModeChange} variant="fullWidth">
        <Tab label="Subscribing" value={PanelMode.SUBSCRIBING} />
        <Tab label="Editing" value={PanelMode.EDITING} />
      </Tabs>
      <h2>{state.panelMode === PanelMode.EDITING ? "Mode -- Editing" : "Mode -- Subscribing"}</h2>
      <div>
        {/* check state and give different outputs */}
        {state.panelMode === PanelMode.EDITING ? (
          // create a 6x4 grid
          <div>
            <Grid container spacing={6} columns={4}>
              <Grid item></Grid>
            </Grid>
            <Button variant="contained" onClick={callServiceWithRequest}>
              Submit
            </Button>
          </div>
        ) : (
          <Grid container spacing={6} columns={4}>
            <Grid item></Grid>
          </Grid>
        )}
      </div>
    </div>
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


const updatePID = (event: React.ChangeEvent<HTMLInputElement>) => {
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

  *\