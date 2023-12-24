import useTheme from "@duke-robotics/theme";
import { Immutable, PanelExtensionContext, RenderState } from "@foxglove/studio";
import { Box, Button, InputAdornment, TextField, ThemeProvider } from "@mui/material";
import Alert from "@mui/material/Alert";
import { JsonViewer } from "@textea/json-viewer";
import { useEffect, useState } from "react";
import { JSX } from "react/jsx-runtime";
import { createRoot } from "react-dom/client";

type CallServicePanelState = {
  serviceName: string;
  request: string;
  response?: unknown;
  error?: Error | undefined;
  colorScheme?: RenderState["colorScheme"];
};

function CallServicePanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [state, setState] = useState<CallServicePanelState>({ serviceName: "", request: "{}" });

  // Update color scheme
  useEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setState((oldState) => ({ ...oldState, colorScheme: renderState.colorScheme }));
      setRenderDone(() => done);
    };
  }, [context]);
  context.watch("colorScheme");

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

  // Close callService with the current state for use in the button
  const callServiceWithRequest = () => {
    void callService(state.serviceName, state.request);
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

        {/* Topic Name Input */}
        <TextField
          label="Topic Name"
          type="text"
          size="small"
          margin="dense"
          fullWidth
          value={state.serviceName}
          onChange={(event) => {
            setState((prevState) => ({
              ...prevState,
              serviceName: event.target.value,
            }));
          }}
          InputProps={{
            startAdornment: <InputAdornment position="start">/</InputAdornment>,
          }}
        />

        {/* Request Input */}
        <TextField
          margin="dense"
          size="small"
          multiline
          label="Request"
          fullWidth
          value={state.request}
          onChange={(event) => {
            setState((prevState) => ({
              ...prevState,
              request: event.target.value,
            }));
          }}
        />

        {/* Publish Once Button */}
        <Button
          fullWidth
          variant="contained"
          disabled={context.callService == undefined || state.serviceName === ""}
          onClick={callServiceWithRequest}
        >
          {`Call Service`}
        </Button>

        <Box>
          <h4>Response</h4>
          <JsonViewer
            rootName={false}
            value={state.error ? { error: state.error.message } : state.response ?? {}}
            indentWidth={2}
            theme={state.colorScheme}
            enableClipboard={false}
            displayDataTypes={false}
          />
        </Box>
      </Box>
    </ThemeProvider>
  );
}

export function initCallServicePanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<CallServicePanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
