import useTheme from "@duke-robotics/theme";
import { Immutable, PanelExtensionContext, RenderState } from "@foxglove/extension";
import { Box, Button, InputAdornment, TextField, ThemeProvider, Typography } from "@mui/material";
import Alert from "@mui/material/Alert";
import { JsonViewer } from "@textea/json-viewer";
import { useEffect, useState } from "react";
import { JSX } from "react/jsx-runtime";
import { createRoot } from "react-dom/client";

type CallServicePanelState = {
  serviceName: string;
  request: string;
  response?: unknown;
  error?: Error;
  colorScheme?: RenderState["colorScheme"];
};

function CallServicePanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  const [state, setState] = useState<CallServicePanelState>(() => {
    const initialState = context.initialState as CallServicePanelState | undefined;

    return {
      serviceName: initialState?.serviceName ?? "",
      request: initialState?.request ?? "{\n\n}",
    };
  });

  // Save state upon change
  useEffect(() => {
    context.saveState(state);
  }, [state, context]);

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
  const callService = () => {
    if (!context.callService) {
      return;
    }

    context.callService(`/${state.serviceName}`, JSON.parse(state.request)).then(
      (response) => {
        JSON.stringify(response); // Attempt serializing the response, to throw an error on failure
        setState((oldState) => ({
          ...oldState,
          response,
          error: undefined,
        }));
      },
      (error) => {
        // Handle service call errors (e.g., service is not advertised)
        setState((oldState) => ({ ...oldState, error: error as Error }));
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

        {/* Service Name Input */}
        <TextField
          label="Service Name"
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

        <Box my={1}>
          {/* Call Service Button */}
          <Button
            fullWidth
            variant="contained"
            disabled={context.callService == undefined || state.serviceName === ""}
            onClick={callService}
          >
            {`Call Service`}
          </Button>
        </Box>

        <Box>
          <Typography variant="subtitle1" gutterBottom>
            Response
          </Typography>
          <JsonViewer
            rootName={false}
            value={state.response}
            indentWidth={2}
            theme={state.colorScheme}
            enableClipboard={true}
            displayDataTypes={false}
          />
        </Box>
      </Box>
    </ThemeProvider>
  );
}

export function initCallServicePanel(context: PanelExtensionContext): () => void {
  context.panelElement.style.overflow = "auto"; // Enable scrolling

  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<CallServicePanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
