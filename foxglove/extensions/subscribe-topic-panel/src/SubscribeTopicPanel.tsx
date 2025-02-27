import useTheme from "@duke-robotics/theme";
import { PanelExtensionContext, RenderState, Topic, MessageEvent, Immutable } from "@foxglove/extension";
import { Autocomplete, Box, TextField, ThemeProvider } from "@mui/material";
import { JsonViewer } from "@textea/json-viewer";
import { useEffect, useState, useMemo } from "react";
import { createRoot } from "react-dom/client";

type SubscribeTopicPanelState = {
  topic?: string;
  colorScheme?: RenderState["colorScheme"];
  topics?: readonly Topic[];
  message?: MessageEvent;
};

function SubscribeTopicPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  // Restore our state from the layout via the context.initialState property.
  const [state, setState] = useState<SubscribeTopicPanelState>(() => {
    return context.initialState as SubscribeTopicPanelState;
  });

  // Get topics
  const topics = useMemo(() => state.topics ?? [], [state.topics]);

  useEffect(() => {
    // Save our state to the layout when the topic changes.
    context.saveState({ topic: state.topic });

    if (state.topic) {
      // Subscribe to the new image topic when a new topic is chosen.
      context.subscribe([{ topic: state.topic }]);
    } else {
      context.unsubscribeAll();
    }
  }, [context, state.topic]);

  // Setup our onRender function and start watching topics and currentFrame for messages.
  useEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setRenderDone(() => done);
      setState((oldState) => ({
        ...oldState,
        topics: renderState.topics,
        colorScheme: renderState.colorScheme,
      }));

      // Save the most recent message on our topic.
      if (renderState.currentFrame && renderState.currentFrame.length > 0) {
        const lastFrame = renderState.currentFrame.at(-1) as MessageEvent;

        setState((oldState) => ({ ...oldState, message: lastFrame }));
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

  const theme = useTheme();
  return (
    <ThemeProvider theme={theme}>
      <Box m={1}>
        {/* Topic Name Input */}
        <Autocomplete
          fullWidth
          options={topics.map((topic) => topic.name)}
          value={state.topic}
          onChange={(_, newValue) => {
            setState((prevState) => ({
              ...prevState,
              topic: newValue ?? undefined,
            }));
            setState((oldState) => ({ ...oldState, message: undefined }));
          }}
          renderInput={(params) => <TextField {...params} label="Topic Name" margin="dense" size="small" />}
        />
        {/* Message */}
        <JsonViewer
          rootName={false}
          value={state.message}
          indentWidth={2}
          theme={state.colorScheme}
          enableClipboard={true}
          displayDataTypes={false}
        />
      </Box>
    </ThemeProvider>
  );
}

export function initSubscribeTopicPanel(context: PanelExtensionContext): () => void {
  context.panelElement.style.overflow = "auto"; // Enable scrolling

  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<SubscribeTopicPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
