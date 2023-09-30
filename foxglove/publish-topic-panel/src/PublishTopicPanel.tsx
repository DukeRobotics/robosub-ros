import { PanelExtensionContext, RenderState } from "@foxglove/studio";
import { useCallback, useEffect, useLayoutEffect, useState } from "react";
import ReactDOM from "react-dom";

type State = {
  topicName: string;
  request: string;
  error?: Error | undefined;
  colorScheme?: RenderState["colorScheme"];
};

function PublishTopicPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [state, setState] = useState<State>({ topicName: "", request: "{}" });

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

  const publishTopic = useCallback(
    async (topicName: string, request: string) => {
      if (!context.advertise) {
        return;
      }
      if (!context.publish) {
        return
      }

      try {
        context.advertise(`/${topicName}`, "std_msgs/ColorRGBA");
        context.publish(`/${topicName}`, JSON.parse(request));

        

        setState((oldState) => ({
          ...oldState,
          error: undefined,
        }));
      } catch (error) {
        setState((oldState) => ({ ...oldState, error: error as Error }));
        console.error(error);
      }
    },
    [context.callService],
  );

  return (
    <div style={{ padding: "1rem" }}>
      <h2>Publish Topic</h2>
      {context.callService == undefined && (
        <p style={{ color: "red" }}>Publishing topics is not supported by this connection</p>
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
          disabled={context.callService == undefined || state.topicName === ""}
          style={{ width: "100%", minHeight: "2rem" }}
          // eslint-disable-next-line @typescript-eslint/no-misused-promises
          onClick={async () => {
            await publishTopic(state.topicName, state.request);
          }}
        >
          {`Publish ${state.topicName}`}
        </button>
      </div>
    </div>
  );
}

export function initPublishTopicPanel(context: PanelExtensionContext): () => void {
  ReactDOM.render(<PublishTopicPanel context={context} />, context.panelElement);

  // Return a function to run when the panel is removed
  return () => {
    ReactDOM.unmountComponentAtNode(context.panelElement);
  };
}
