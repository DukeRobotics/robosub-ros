import { PanelExtensionContext } from "@foxglove/studio";
import Alert from "@mui/material/Alert";
import Box from "@mui/material/Box";
import Button from "@mui/material/Button/Button";
import { useState } from "react";
import { createRoot } from "react-dom/client";

type State = {
  error?: Error | undefined;
  controlsEnabled: boolean;
};

function ToggleControlsPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [state, setState] = useState<State>({ controlsEnabled: false });

  // Call the /enable_controls service to toggle controls
  const toggleControls = async () => {
    if (!context.callService) {
      return;
    }

    const serviceName = "/enable_controls";
    const request = { data: !state.controlsEnabled };

    await context.callService(serviceName, request).then(
      (response) => {
        setState({ ...state, controlsEnabled: !state.controlsEnabled });
        JSON.stringify(response);
        setState((oldState) => ({
          ...oldState,
          error: undefined,
        }));
      },
      (error) => {
        setState((oldState) => ({
          ...oldState,
          error: error as Error,
        }));
      },
    );
  };

  return (
    <div style={{ padding: "1rem" }}>
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

      <Button
        variant="contained"
        color={state.controlsEnabled ? "error" : "success"}
        onClick={() => {
          void toggleControls();
        }}
        disabled={context.callService == undefined}
      >
        {state.controlsEnabled ? "Disable" : "Enable"}
      </Button>
    </div>
  );
}

export function initToggleControlsPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<ToggleControlsPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
