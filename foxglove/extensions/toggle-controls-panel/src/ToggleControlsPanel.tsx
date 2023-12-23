import { StdSrvsSetBoolRequest, StdSrvsSetBoolResponse } from "@duke-robotics/defs/types";
import useTheme from "@duke-robotics/theme";
import { ThemeProvider } from "@emotion/react";
import { PanelExtensionContext } from "@foxglove/studio";
import Alert from "@mui/material/Alert";
import Box from "@mui/material/Box";
import Button from "@mui/material/Button/Button";
import { useState } from "react";
import { createRoot } from "react-dom/client";

// Define the service name for enabling/disabling controls
const ENABLE_CONTROLS_SERVICE = "/enable_controls";

type ToggleControlsPanel = {
  error?: Error | undefined; // Error object if service call fails
  controlsEnabled: boolean; // Current state of controls
};

function ToggleControlsPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [state, setState] = useState<ToggleControlsPanel>({ controlsEnabled: false });

  // Call the /enable_controls service to toggle controls
  const toggleControls = () => {
    // Check if service calling is supported by the context
    if (!context.callService) {
      console.error("Calling services is not supported by this connection");
      return;
    }

    // Request payload to toggle controls
    const request: StdSrvsSetBoolRequest = { data: !state.controlsEnabled };

    // Make the service call
    context.callService(ENABLE_CONTROLS_SERVICE, request).then(
      (response) => {
        const typedResponse = response as StdSrvsSetBoolResponse;

        // Update the state based on the service response
        // If the service responds with failure, display the response message as an error
        const error = typedResponse.success ? undefined : Error(typedResponse.message);
        setState({ error, controlsEnabled: !state.controlsEnabled });
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

        {/* Toggle button */}
        <Button
          variant="contained"
          color={state.controlsEnabled ? "error" : "success"}
          onClick={toggleControls}
          disabled={context.callService == undefined}
        >
          {state.controlsEnabled ? "Disable Controls" : "Enable Controls"}
        </Button>
      </Box>
    </ThemeProvider>
  );
}

export function initToggleControlsPanel(context: PanelExtensionContext): () => void {
  context.panelElement.style.overflow = "auto"; // Enable scrolling

  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<ToggleControlsPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
