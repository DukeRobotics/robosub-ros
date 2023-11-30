import { Immutable, PanelExtensionContext, RenderState, MessageEvent } from "@foxglove/studio";
import { Alert, Tab, Tabs, TableBody, TableContainer, TableCell, TableRow, TableHead, Table } from "@mui/material";
// X import { JsonViewer } from "@textea/json-viewer";
import { useEffect, useLayoutEffect, useState } from "react";
import React = require("react");
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
  LOOP_POSITION: { ...axes },
  LOOP_VELOCITY: { ...axes },
};

const topicName = "/current_pid";
const serviceName = "/set_pid";
const pidTypeToUpdate = "LOOP_POSITION";

if (!pid[pidTypeToUpdate]) {
  pid[pidTypeToUpdate] = {};
}

enum PanelMode {
  SUBSCRIBING,
  EDITING,
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
        <TableContainer>
          <Table size="small" aria-label="simple table">
            <TableHead>
              <TableRow>
                <TableCell align="center" width="50px"></TableCell>
                <TableCell align="center" width="50px">
                  Gain_KP
                </TableCell>
                <TableCell align="center" width="50px">
                  Gain_KI
                </TableCell>
                <TableCell align="center" width="50px">
                  Gain_KD
                </TableCell>
                <TableCell align="center" width="50px">
                  Gain_FF
                </TableCell>
              </TableRow>
            </TableHead>
            <TableBody>
              {pid[pidTypeToUpdate] &&
                Object.entries(pid[pidTypeToUpdate]).map(([axis, g]) => (
                  <TableRow key={axis}>
                    <TableCell width="50px">{axis}</TableCell>
                    {Object.values(g).map((gain, index) => (
                      <TableCell key={index} width="50px">
                        {gain}
                      </TableCell>
                    ))}
                  </TableRow>
                ))}
            </TableBody>
          </Table>
        </TableContainer>
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
