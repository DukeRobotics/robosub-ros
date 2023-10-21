import { PanelExtensionContext, RenderState, Topic, MessageEvent } from "@foxglove/studio";
import { useLayoutEffect, useEffect, useState, useRef, useMemo } from "react";
import ReactDOM from "react-dom";
import ReactJson from "react-json-view";
import Alert from '@mui/material/Alert';
import Grid from "@mui/material/Unstable_Grid2";
import Paper from '@mui/material/Paper';
import { styled } from '@mui/material/styles';

const Item = styled(Paper)(({ theme }) => ({
  backgroundColor: theme.palette.mode === 'dark' ? '#1A2027' : '#fff',
  ...theme.typography.body2,
  padding: theme.spacing(1),
  textAlign: 'center',
  color: theme.palette.text.secondary,
}));

type ThrusterSpeeds = {
  frontLeft: number,
  frontRight: number,
  backLeft: number,
  backRight: number,
  bottomFrontLeft: number,
  bottomFrontRight: number,
  bottomBackLeft: number,
  bottomBackRight: number
}

type State = {
  topic?: string;
  colorScheme?: RenderState["colorScheme"];
  thrusterSpeeds: ThrusterSpeeds;
};

function ThrusterSpeedsSubscriber({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [topics, setTopics] = useState<readonly Topic[] | undefined>();
  const [message, setMessage] = useState<any>();


  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  // Restore our state from the layout via the context.initialState property.

  const thrusters = ["frontLeft", "frontRight", "backLeft", "backRight", 
                     "bottomFrontLeft", "bottomFrontRight", "bottomBackLeft", "bottomBackRight"];

  const [state, setState] = useState<State>({
    thrusterSpeeds: {
      frontLeft: 0,
      frontRight: 0,
      backLeft: 0,
      backRight: 0,
      bottomFrontLeft: 0,
      bottomFrontRight: 0,
      bottomBackLeft: 0,
      bottomBackRight: 0
    } 
  });

  // Get topics
  const imageTopics = useMemo(
    () => (topics ?? []),
    [topics],
  );

  useEffect(() => {
    // Save our state to the layout when the topic changes.
    context.saveState({ topic: state.topic });

    if (state.topic) {
      // Subscribe to the new image topic when a new topic is chosen.
      context.subscribe([{ topic: state.topic }]);
    }
  }, [context, state.topic]);

  // Choose our first available image topic as a default once we have a list of topics available.
  useEffect(() => {
    if (state.topic == undefined) {
      // setState({ topic: imageTopics[0]?.name });
    }
  }, [state.topic, imageTopics]);

  // Setup our onRender function and start watching topics and currentFrame for messages.
  useLayoutEffect(() => {
    context.onRender = (renderState: RenderState, done) => {
      setRenderDone(() => done);
      setTopics(renderState.topics);
      setState((oldState) => ({ ...oldState, colorScheme: renderState.colorScheme }));
      
      // Save the most recent message on our topic.
      if (renderState.currentFrame && renderState.currentFrame.length > 0) {
        setMessage(renderState.currentFrame[renderState.currentFrame.length - 1]);
      }
    };

    context.watch("topics");
    context.watch("currentFrame");
    context.watch("colorScheme");

  }, [context]);

  // Call our done function at the end of each render.
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  return (
    <div style={{ height: "100%", padding: "1rem" }}>
      <h2>Subscribe Topic</h2>
      {context.subscribe == undefined && (
        <Alert variant="filled" severity="error">Subscribing to topics is not supported by this connection</Alert>
      )}
      <div>
        <label>Choose a topic to display: </label>
        <Grid container rowSpacing={1} columnSpacing={{ xs: 1, sm: 2, md: 3 }}>
          {thrusters.map((thruster) => (
            <Grid xs={6}>
              <Item>{state.thrusterSpeeds[thruster]}</Item>
            </Grid>
          ))}
        </Grid>

        <ReactJson
          name={null}
          src={message}
          indentWidth={2}
          theme={state.colorScheme === "dark" ? "monokai" : "rjv-default"}
          enableClipboard={false}
          displayDataTypes={false}
        />

      </div>
    </div>
  );
}

export function initThrusterSpeedsSubscriber(context: PanelExtensionContext): () => void {
  ReactDOM.render(<ThrusterSpeedsSubscriber context={context} />, context.panelElement);

  // Return a function to run when the panel is removed
  return () => {
    ReactDOM.unmountComponentAtNode(context.panelElement);

  };
}
