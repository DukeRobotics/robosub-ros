import { PanelExtensionContext, RenderState } from "@foxglove/studio";
import Button from "@mui/material/Button/Button";
import CheckCircleOutlineIcon from '@mui/icons-material/CheckCircleOutline';
import HighlightOffIcon from '@mui/icons-material/HighlightOff';
import { useCallback, useEffect, useLayoutEffect, useState } from "react";
import ReactDOM from "react-dom";
import ReactJson from "react-json-view";


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

  useLayoutEffect(() => {
    context.onRender = (renderState: RenderState, done) => {
      setState((oldState) => ({ ...oldState, colorScheme: renderState.colorScheme }));
      setRenderDone(() => done);
    };
  }, [context]);

  context.watch("colorScheme");

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const toggleControls = async () => {
    if (!context.callService) {
      return;
    }

    const serviceName = "/enable_controls";
    const request = { "data": !state.controlsEnabled };

    setState({ ...state, controlsEnabled: !state.controlsEnabled })

    try {
      const response = await context.callService(serviceName, request);
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

  }

  return (
    <div style={{ padding: "1rem" }}>
      <h2>Toggle Controls</h2>
      {context.callService == undefined && (
        <p style={{ color: "red" }}>Calling services is not supported by this connection</p>
      )}

      <Button
        variant="contained"
        color={state.controlsEnabled ? "error" : "success"}
        endIcon={state.controlsEnabled ? <HighlightOffIcon /> : <CheckCircleOutlineIcon />}
        onClick={async () => { await toggleControls(); }}
      >
        {state.controlsEnabled ? "Disable Controls" : "Enable Controls"}
      </Button>

      <div>
        <h4>Response</h4>
        <ReactJson
          name={null}
          src={state.error ? { error: state.error.message } : state.response ?? {}}
          indentWidth={2}
          enableClipboard={false}
          theme={state.colorScheme === "dark" ? "monokai" : "rjv-default"}
          displayDataTypes={false}
        />
      </div>
    </div>
  );
}

export function initToggleControlsPanel(context: PanelExtensionContext): () => void {
  ReactDOM.render(<ToggleControlsPanel context={context} />, context.panelElement);

  // Return a function to run when the panel is removed
  return () => {
    ReactDOM.unmountComponentAtNode(context.panelElement);
  };
}
