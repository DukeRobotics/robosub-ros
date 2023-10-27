import { PanelExtensionContext, RenderState, Topic, Immutable, MessageEvent } from "@foxglove/studio";
import { TextField } from "@mui/material";
import Grid from "@mui/material/Unstable_Grid2";
import { useLayoutEffect, useEffect, useState, useMemo } from "react";
import { createRoot } from "react-dom/client";

type ThrusterSpeeds = {
  frontLeft: number;
  frontRight: number;
  backLeft: number;
  backRight: number;
  bottomFrontLeft: number;
  bottomFrontRight: number;
  bottomBackLeft: number;
  bottomBackRight: number;
};

type State = {
  topic?: string;
  colorScheme?: RenderState["colorScheme"];
  thrusterSpeeds: ThrusterSpeeds;
};

function ThrusterSpeedsSubscriber({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [topics, setTopics] = useState<readonly Topic[] | undefined>();
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  // Restore our state from the layout via the context.initialState property.
  const [state, setState] = useState<State>(() => {
    const initialState = context.initialState as State;
    initialState.topic = "/offboard/thruster_speeds";
    initialState.thrusterSpeeds = {
      frontLeft: 0,
      frontRight: 0,
      backLeft: 0,
      backRight: 0,
      bottomFrontLeft: 0,
      bottomFrontRight: 0,
      bottomBackLeft: 0,
      bottomBackRight: 0,
    };

    return initialState;
  });

  const thrusters: (keyof ThrusterSpeeds)[] = [
    "frontLeft",
    "frontRight",
    "backLeft",
    "backRight",
    "bottomFrontLeft",
    "bottomFrontRight",
    "bottomBackLeft",
    "bottomBackRight",
  ];

  // Get topics
  const imageTopics = useMemo(() => topics ?? [], [topics]);

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
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setRenderDone(() => done);
      setTopics(renderState.topics);
      setState((oldState) => ({ ...oldState, colorScheme: renderState.colorScheme }));

      const thrustersInOrder: (keyof ThrusterSpeeds)[] = [
        "bottomFrontLeft",
        "frontLeft",
        "frontRight",
        "bottomFrontRight",
        "bottomBackLeft",
        "backLeft",
        "bottomBackRight",
        "backRight",
      ];

      // Save the most recent message on our topic.
      if (renderState.currentFrame && renderState.currentFrame.length > 0) {
        const latestFrame = renderState.currentFrame[renderState.currentFrame.length - 1] as MessageEvent<any>;
        let newSpeeds: ThrusterSpeeds = {
          frontLeft: 0,
          frontRight: 0,
          backLeft: 0,
          backRight: 0,
          bottomFrontLeft: 0,
          bottomFrontRight: 0,
          bottomBackLeft: 0,
          bottomBackRight: 0,
        };

        thrustersInOrder.forEach((thruster: keyof ThrusterSpeeds, index) => {
          newSpeeds = { ...newSpeeds, [thruster]: latestFrame.message?.speeds[index] };
        });
        setState((oldState) => ({ ...oldState, thrusterSpeeds: newSpeeds }));
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
      <Grid container rowSpacing={1} columnSpacing={1}>
        {thrusters.map((thruster) => (
          <Grid key={thruster} xs={6}>
            <TextField
              key={thruster}
              id={thruster}
              label={thruster}
              size="small"
              variant="filled"
              value={state.thrusterSpeeds[thruster]}
              InputProps={{ readOnly: true }}
            />
          </Grid>
        ))}
      </Grid>
    </div>
  );
}

export function initThrusterSpeedsSubscriber(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<ThrusterSpeedsSubscriber context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
