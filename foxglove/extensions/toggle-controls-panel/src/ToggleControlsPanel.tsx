import { StdSrvsSetBoolRequest, StdMsgsBool } from "@duke-robotics/defs/types";
import useTheme from "@duke-robotics/theme";
import { ThemeProvider } from "@emotion/react";
import { PanelExtensionContext, RenderState, Immutable, MessageEvent } from "@foxglove/extension";
import Alert from "@mui/material/Alert";
import Box from "@mui/material/Box";
import Button from "@mui/material/Button/Button";
import { useState, useEffect } from "react";
import { createRoot } from "react-dom/client";

// Define the service name for enabling/disabling controls
const ENABLE_CONTROLS_SERVICE = "/controls/enable";
const CONTROLS_STATUS_TOPIC = "/controls/status";

type ToggleControlsPanel = {
  error?: Error | undefined; // Error object if service call fails
  controlsEnabled: boolean; // Current state of controls
};

function ToggleControlsPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [state, setState] = useState<ToggleControlsPanel>({ controlsEnabled: false });
  const [renderDone, setRenderDone] = useState<() => void | undefined>();

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // useEffect hook for subscribing to CONTROLS_STATUS_TOPIC
  useEffect(() => {
    context.saveState({ topic: CONTROLS_STATUS_TOPIC });
    context.subscribe([{ topic: CONTROLS_STATUS_TOPIC }]);
  }, [context]);

  // useEffect hook for rendering and watching renderState. Saves the values from the most recent message.
  useEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setRenderDone(() => done);

      // Save the most recent message on our topic.
      if (renderState.currentFrame && renderState.currentFrame.length > 0) {
        const latestFrame = renderState.currentFrame[renderState.currentFrame.length - 1] as MessageEvent<StdMsgsBool>;
        setState((oldState) => ({ ...oldState, controlsEnabled: latestFrame.message.data as boolean }));
      }
    };

    context.watch("currentFrame");
  }, [context]);

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
    context.callService(ENABLE_CONTROLS_SERVICE, request).catch((error) => {
      // Handle service call errors (e.g., service is not advertised)
      setState((oldState) => ({
        ...oldState,
        error: error as Error,
      }));
    });
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
                Calling services is not supported by this connection.
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
          fullWidth
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
