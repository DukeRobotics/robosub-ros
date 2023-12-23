import { AllDatatypeMapsType, allDatatypeMaps } from "@duke-robotics/defs/datatype_maps";
import useTheme from "@duke-robotics/theme";
import { Immutable, PanelExtensionContext, RenderState } from "@foxglove/studio";
import { Box, Button, Grid, InputAdornment, MenuItem, TextField, ThemeProvider } from "@mui/material";
import Alert from "@mui/material/Alert";
import { useEffect, useState } from "react";
import { createRoot } from "react-dom/client";

type PublishTopicPanelState = {
  topicName: string;
  request: string;
  schemaType: keyof AllDatatypeMapsType;
  schemaName?: string;
  publishRate: number; // Hz
  // If publishing, holds the NodeJS.Timeout object used to publish messages at a constant rate, otherwise null
  repeatPublish: NodeJS.Timeout | null;
  invalidRate: boolean;
};

function PublishTopicPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [state, setState] = useState<PublishTopicPanelState>({
    topicName: "",
    request: "{}",
    schemaType: "ros1",
    publishRate: 10,
    repeatPublish: null,
    invalidRate: false,
  });

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

  // Pubish a request with a given schema to a topic
  const publishTopic = () => {
    if (!context.advertise || !context.publish || state.schemaName == undefined) {
      return;
    }

    context.advertise(`/${state.topicName}`, state.schemaName, {
      // @ts-expect-error: state.schemaName will always be a valid key of allDatatypeMaps[state.schemaType]
      datatypes: allDatatypeMaps[state.schemaType][state.schemaName],
    });
    context.publish(`/${state.topicName}`, JSON.parse(state.request));
  };

  // Function to toggle interval for publishing
  const toggleInterval = () => {
    if (state.repeatPublish == null) {
      setState((oldState) => ({
        ...oldState,
        repeatPublish: setInterval(() => {
          publishTopic();
        }, 1000 / state.publishRate), // Hz to ms
      }));
    } else {
      clearInterval(state.repeatPublish);
      setState((oldState) => ({
        ...oldState,
        repeatPublish: null,
      }));
    }
  };

  const handleRateChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    const value = Number(event.target.value);

    if (value > 0) {
      setState({ ...state, publishRate: Number(event.target.value), invalidRate: false });
    } else {
      setState({ ...state, invalidRate: true });
    }
  };

  const theme = useTheme();
  return (
    <ThemeProvider theme={theme}>
      <Box m={1}>
        {(context.advertise == undefined || context.publish == undefined) && (
          <Alert variant="filled" severity="error">
            Publishing topics is not supported by this connection
          </Alert>
        )}

        <TextField
          margin="dense"
          size="small"
          type="text"
          label="Topic Name"
          fullWidth
          value={state.topicName}
          onChange={(event) => {
            setState({ ...state, topicName: event.target.value });
          }}
          InputProps={{
            startAdornment: <InputAdornment position="start">/</InputAdornment>,
          }}
        />

        <Grid container spacing={1}>
          <Grid item xs={4}>
            <TextField
              select
              label="Schema Type"
              margin="dense"
              size="small"
              fullWidth
              value={state.schemaType}
              onChange={(event) => {
                setState({ ...state, schemaType: event.target.value as "ros1" | "custom_msgs" });
              }}
            >
              {Object.entries(allDatatypeMaps).map(([name, _]) => (
                <MenuItem key={name} value={name}>
                  {name}
                </MenuItem>
              ))}
            </TextField>
          </Grid>
          <Grid item xs={8}>
            <TextField
              select
              label="Schema Name"
              margin="dense"
              size="small"
              fullWidth
              onChange={(event) => {
                setState({ ...state, schemaName: event.target.value });
              }}
            >
              {Object.entries(allDatatypeMaps[state.schemaType]).map(([name, _]) => (
                <MenuItem key={name} value={name}>
                  {name}
                </MenuItem>
              ))}
            </TextField>
          </Grid>
        </Grid>

        <TextField
          margin="dense"
          size="small"
          error={state.invalidRate}
          helperText={state.invalidRate ? "Rate must be positive." : ""}
          label="Rate (Hz)"
          defaultValue={state.publishRate}
          type="number"
          fullWidth
          onChange={handleRateChange}
          InputProps={{
            endAdornment: <InputAdornment position="end">Hz</InputAdornment>,
          }}
        />

        <TextField
          margin="dense"
          multiline
          label="Request"
          fullWidth
          value={state.request}
          onChange={(event) => {
            setState({ ...state, request: event.target.value });
          }}
        />

        <Box my={1}>
          <Grid container spacing={1}>
            <Grid item xs={6}>
              <Button
                fullWidth
                variant="contained"
                disabled={
                  context.advertise == undefined ||
                  context.publish == undefined ||
                  state.topicName === "" ||
                  state.schemaName == undefined ||
                  state.repeatPublish != null
                }
                onClick={publishTopic}
              >
                {`Publish Once`}
              </Button>
            </Grid>
            <Grid item xs={6}>
              <Button
                fullWidth
                variant="contained"
                disabled={
                  context.advertise == undefined ||
                  context.publish == undefined ||
                  state.topicName === "" ||
                  state.schemaName == undefined ||
                  state.invalidRate
                }
                onClick={toggleInterval}
                color={state.repeatPublish == null ? "success" : "error"}
              >
                {state.repeatPublish == null ? "Start Publish Loop" : "Stop Publish Loop"}
              </Button>
            </Grid>
          </Grid>
        </Box>
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
