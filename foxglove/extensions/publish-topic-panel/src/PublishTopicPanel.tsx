import { AllDatatypeMapsType, allDatatypeMaps } from "@duke-robotics/defs/datatype_maps";
import useTheme from "@duke-robotics/theme";
import { PanelExtensionContext } from "@foxglove/extension";
import { Autocomplete, Box, Button, Grid, InputAdornment, TextField, ThemeProvider } from "@mui/material";
import Alert from "@mui/material/Alert";
import { useEffect, useState } from "react";
import { createRoot } from "react-dom/client";

type PublishTopicPanelState = {
  topicName: string;
  request: string;
  schemaType: keyof AllDatatypeMapsType;
  schemaName?: string; // Should be a key of allDatatypeMaps[state.schemaType]
  publishRate: number; // Hz
  // If publishing, holds the NodeJS.Timeout object used to publish messages at a constant rate, otherwise null
  repeatPublish: NodeJS.Timeout | null;
  invalidRate: boolean; // True if the rate is invalid (<= 0)
  error?: Error;
};

function PublishTopicPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [state, setState] = useState<PublishTopicPanelState>(() => {
    const initialState = context.initialState as PublishTopicPanelState | undefined;

    return {
      topicName: initialState?.topicName ?? "",
      request: initialState?.request ?? "{\n\n}",
      schemaType: (initialState?.schemaType ?? "ros1") as keyof AllDatatypeMapsType,
      schemaName: initialState?.schemaName ?? undefined,
      publishRate: initialState?.publishRate ?? 1,
      repeatPublish: null,
      invalidRate: false,
    };
  });

  // Save state upon change
  useEffect(() => {
    context.saveState(state);
  }, [state, context]);

  // Pubish a request with a given schema to a topic
  const publishTopic = () => {
    if (!context.advertise || !context.publish || state.schemaName == undefined) {
      return;
    }

    context.advertise(`/${state.topicName}`, state.schemaName, {
      // @ts-expect-error: state.schemaName will always be a valid key of allDatatypeMaps[state.schemaType]
      datatypes: allDatatypeMaps[state.schemaType][state.schemaName],
    });

    try {
      context.publish(`/${state.topicName}`, JSON.parse(state.request));
      setState((prevState) => ({
        ...prevState,
        error: undefined,
      }));
    } catch (e) {
      setState((prevState) => ({
        ...prevState,
        error: e as Error, // Catch JSON parse errors
      }));
    }
  };

  // Function to toggle interval for publishing
  const toggleInterval = () => {
    if (state.repeatPublish == null) {
      setState((prevState) => ({
        ...prevState,
        repeatPublish: setInterval(() => {
          publishTopic();
        }, 1000 / state.publishRate), // Hz to ms
      }));
    } else {
      clearInterval(state.repeatPublish);
      setState((prevState) => ({
        ...prevState,
        repeatPublish: null,
      }));
    }
  };

  // Validate rate input
  const handleRateChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    const value = Number(event.target.value);

    if (value > 0) {
      setState((prevState) => ({
        ...prevState,
        publishRate: Number(event.target.value),
        invalidRate: false,
      }));
    } else {
      setState((prevState) => ({
        ...prevState,
        invalidRate: true,
      }));
    }
  };

  const theme = useTheme();
  return (
    <ThemeProvider theme={theme}>
      <Box m={1}>
        {/* Error messages */}
        {(context.advertise == undefined || context.publish == undefined || state.error != undefined) && (
          <Box mb={1}>
            {(context.advertise == undefined || context.publish == undefined) && (
              <Alert variant="filled" severity="error">
                Publishing topics is not supported by this connection.
              </Alert>
            )}
            {state.error != undefined && (
              <Alert variant="filled" severity="error">
                {state.error.message}
              </Alert>
            )}
          </Box>
        )}

        <Grid container spacing={1}>
          <Grid item xs={8}>
            {/* Topic Name Input */}
            <TextField
              label="Topic Name"
              type="text"
              size="small"
              margin="dense"
              fullWidth
              value={state.topicName}
              disabled={state.repeatPublish != null}
              onChange={(event) => {
                setState((prevState) => ({
                  ...prevState,
                  topicName: event.target.value,
                }));
              }}
              InputProps={{
                startAdornment: <InputAdornment position="start">/</InputAdornment>,
              }}
            />
          </Grid>
          <Grid item xs={4}>
            {/* Rate Input */}
            <TextField
              label="Rate"
              type="number"
              size="small"
              margin="dense"
              fullWidth
              error={state.invalidRate}
              helperText={state.invalidRate ? "Rate must be positive." : ""}
              defaultValue={state.publishRate}
              disabled={state.repeatPublish != null}
              onChange={handleRateChange}
              InputProps={{
                endAdornment: <InputAdornment position="end">Hz</InputAdornment>,
              }}
            />
          </Grid>
        </Grid>

        <Grid container spacing={1}>
          <Grid item xs={4}>
            {/* Schema Type Input */}
            <Autocomplete
              fullWidth
              options={Object.keys(allDatatypeMaps)}
              value={state.schemaType}
              disabled={state.repeatPublish != null}
              onChange={(_, newValue) => {
                // Don't allow undefined since we need to use schema type as a key to get the schema names
                setState((prevState) => ({
                  ...prevState,
                  schemaType: (newValue ?? state.schemaType) as keyof AllDatatypeMapsType,
                }));
              }}
              renderInput={(params) => <TextField {...params} label="Schema Type" margin="dense" size="small" />}
            />
          </Grid>
          <Grid item xs={8}>
            {/* Schema Name Input */}
            <Autocomplete
              fullWidth
              options={Object.keys(allDatatypeMaps[state.schemaType])}
              value={state.schemaName}
              disabled={state.repeatPublish != null}
              onChange={(_, newValue) => {
                setState((prevState) => ({
                  ...prevState,
                  schemaName: newValue ?? undefined,
                }));
              }}
              renderInput={(params) => <TextField {...params} label="Schema Name" margin="dense" size="small" />}
            />
          </Grid>
        </Grid>

        {/* Request Input */}
        <TextField
          margin="dense"
          size="small"
          multiline
          label="Request"
          fullWidth
          value={state.request}
          disabled={state.repeatPublish != null}
          onChange={(event) => {
            setState((prevState) => ({
              ...prevState,
              request: event.target.value,
            }));
          }}
        />

        <Box my={1}>
          <Grid container spacing={1}>
            <Grid item xs={6}>
              {/* Publish Once Button */}
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
              {/* Rate Publishing Toggle */}
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
                {state.repeatPublish == null ? "Start Publishing" : "Stop Publishing"}
              </Button>
            </Grid>
          </Grid>
        </Box>
      </Box>
    </ThemeProvider>
  );
}

export function initPublishTopicPanel(context: PanelExtensionContext): () => void {
  context.panelElement.style.overflow = "auto"; // Enable scrolling

  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<PublishTopicPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
