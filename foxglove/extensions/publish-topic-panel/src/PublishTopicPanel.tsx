import theme from "@duke-robotics/theme";
import { Immutable, PanelExtensionContext, RenderState } from "@foxglove/studio";
import { Box, ThemeProvider } from "@mui/material";
import Alert from "@mui/material/Alert";
import { useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

type State = {
  topicName: string;
  request: string;
  schemaName: string;
  error?: Error | undefined;
  colorScheme?: RenderState["colorScheme"];
};

function PublishTopicPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [state, setState] = useState<State>({ topicName: "", request: "{}", schemaName: "" });

  // Update color scheme
  useLayoutEffect(() => {
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

  // Pubish a request with a given schema to a topic
  const publishTopic = (topicName: string, request: string, schemaName: string) => {
    if (!context.advertise || !context.publish) {
      return;
    }

    context.advertise(`/${topicName}`, schemaName);
    context.publish(`/${topicName}`, JSON.parse(request));
  };

  // Close publishTopic with the current state for use in the button
  const publishTopicWithRequest = () => {
    publishTopic(state.topicName, state.request, state.schemaName);
  };

  return (
    <ThemeProvider theme={theme}>
      <Box m={1}>
        <h2>Publish Topic</h2>
        {(context.advertise == undefined || context.publish == undefined) && (
          <Alert variant="filled" severity="error">
            Publishing topics is not supported by this connection
          </Alert>
        )}

        <h4>Topic Name</h4>
        <div>
          <input
            type="text"
            placeholder="Enter topic name"
            style={{ width: "100%" }}
            value={state.topicName}
            onChange={(event) => {
              setState({ ...state, topicName: event.target.value });
            }}
          />
        </div>
        <h4>Schema Name</h4>
        <div>
          <input
            type="text"
            placeholder="Enter schema name"
            style={{ width: "100%" }}
            value={state.schemaName}
            onChange={(event) => {
              setState({ ...state, schemaName: event.target.value });
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
            disabled={context.advertise == undefined || context.publish == undefined || state.topicName === ""}
            style={{ width: "100%", minHeight: "2rem" }}
            onClick={publishTopicWithRequest}
          >
            {`Publish to ${state.topicName}`}
          </button>
        </div>
      </Box>
    </ThemeProvider>
  );
}

export function initPublishTopicPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<PublishTopicPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
