import { PanelExtensionContext, RenderState, Immutable } from "@foxglove/studio";
import Alert from "@mui/material/Alert";
import Button from "@mui/material/Button/Button";
import { useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

type State = {
  serviceName: string;
  request: string;
  response?: unknown;
  error?: Error | undefined;
  colorScheme?: RenderState["colorScheme"];
  controlsEnabled: boolean;
};

function ToggleControlsPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [state, setState] = useState<State>({ serviceName: "", request: "{}", controlsEnabled: false });

  // Update color scheme
  useLayoutEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setRenderDone(() => done);
      setState((oldState) => ({ ...oldState, colorScheme: renderState.colorScheme }));
    };
  }, [context]);
  context.watch("colorScheme");

  // Call our done function at the end of each render
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

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
        console.log(response);
        setState((oldState) => ({
          ...oldState,
          response,
          error: undefined,
        }));
      },
      (error) => {
        setState((oldState) => ({
          ...oldState,
          response: undefined,
          error: error as Error,
        }));
      },
    );
  };

  return (
    <div style={{ padding: "1rem" }}>
      {context.callService == undefined && (
        <Alert variant="filled" severity="error" style={{ marginBottom: 20 }}>
          Calling services is not supported by this connection
        </Alert>
      )}
      {state.error != undefined && (
        <Alert variant="filled" severity="error" style={{ marginBottom: 20 }}>
          {state.error.message}
        </Alert>
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
