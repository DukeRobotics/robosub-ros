import { Immutable, PanelExtensionContext, RenderState } from "@foxglove/studio";
import Alert from "@mui/material/Alert";
import { JsonViewer } from "@textea/json-viewer";
import { useCallback, useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

type State = {
  serviceName: string;
  request: string;
  response?: unknown;
  error?: Error | undefined;
  colorScheme?: RenderState["colorScheme"];
};

function CallServicePanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [state, setState] = useState<State>({ serviceName: "", request: "{}" });

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

  const callService = useCallback(
    (serviceName: string, request: string) => {
      if (!context.callService) {
        return;
      }

      try {
        const response = context.callService(serviceName, JSON.parse(request));
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
    },
    [context],
  );

  return (
    <div style={{ padding: "1rem" }}>
      <h2>Call Service</h2>
      {context.callService == undefined && (
        <Alert variant="filled" severity="error">
          Calling services is not supported by this connection
        </Alert>
      )}

      <h4>Service Name</h4>
      <div>
        <input
          type="text"
          placeholder="Enter service name"
          style={{ width: "100%" }}
          value={state.serviceName}
          onChange={(event) => {
            setState({ ...state, serviceName: event.target.value });
          }}
        />
      </div>
      <h4>Request</h4>
      <div>
        <textarea
          style={{ width: "100%", minHeight: "3rem" }}
          value={state.request}
          onChange={(event) => {
            setState({ ...state, request: event.target.value });
          }}
        />
      </div>
      <div>
        <button
          disabled={context.callService == undefined || state.serviceName === ""}
          style={{ width: "100%", minHeight: "2rem" }}
          onClick={() => {
            callService(state.serviceName, state.request);
          }}
        >
          {`Call ${state.serviceName}`}
        </button>
      </div>

      <div>
        <h4>Response</h4>
        <JsonViewer
          rootName={false}
          value={state.error ? { error: state.error.message } : state.response ?? {}}
          indentWidth={2}
          theme={state.colorScheme}
          enableClipboard={false}
          displayDataTypes={false}
        />
      </div>
    </div>
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
