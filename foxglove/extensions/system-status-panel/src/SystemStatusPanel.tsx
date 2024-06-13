import { CustomMsgsSystemUsage, StdMsgsFloat64 } from "@duke-robotics/defs/types";
import useTheme from "@duke-robotics/theme";
import { PanelExtensionContext, RenderState, MessageEvent, Immutable } from "@foxglove/studio";
import {
  Box,
  Typography,
  Paper,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableRow,
  ThemeProvider,
} from "@mui/material";
import { useEffect, useState } from "react";
import { createRoot } from "react-dom/client";

// Topic to watch for system usage data
const SYSTEM_USAGE_TOPIC = "/system/usage";
const VOLTAGE_TOPIC = "/sensors/voltage";
const HUMIDITY_TOPIC = "/sensors/humidity";
const TEMPERATURE_TOPIC = "/sensors/temperature";

type SystemStatusPanelState = {
  cpuUsage?: number;
  ramUsage?: number;
  voltage?: number;
  humidity?: number;
  temperature?: number;
};

function SystemStatusPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [state, setState] = useState<SystemStatusPanelState>({});

  context.subscribe([
    { topic: SYSTEM_USAGE_TOPIC },
    { topic: VOLTAGE_TOPIC },
    { topic: HUMIDITY_TOPIC },
    { topic: TEMPERATURE_TOPIC },
  ]);

  // Define values in table
  const rows = [
    {
      statusName: "CPU",
      value: state.cpuUsage,
      suffix: "%",
      warn: () => state.cpuUsage != undefined && state.cpuUsage >= 90,
    },
    {
      statusName: "RAM",
      value: state.ramUsage,
      suffix: "%",
      warn: () => state.ramUsage != undefined && state.ramUsage >= 90,
    },
    {
      statusName: "Voltage",
      value: state.voltage,
      suffix: "V",
      warn: () => {
        return state.voltage != undefined && state.voltage <= 15;
      },
    },
    {
      statusName: "Humidity",
      value: state.humidity,
      suffix: "%",
      warn: () => {
        return state.humidity != undefined && state.humidity >= 80;
      },
    },
    {
      statusName: "Temperature",
      value: state.temperature,
      suffix: "F",
      warn: () => {
        return state.temperature != undefined && state.temperature >= 100;
      },
    },
  ];

  // Watch system usage topic and update state
  useEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setRenderDone(() => done);

      // Reset state when the user seeks the video
      if (renderState.didSeek ?? false) {
        setState({});
      }

      // Update system usage state
      if (renderState.currentFrame && renderState.currentFrame.length > 0) {
        const latestFrame = renderState.currentFrame.at(-1) as MessageEvent;
        if (latestFrame.topic === SYSTEM_USAGE_TOPIC) {
          const systemUsagelatestFrame = latestFrame as MessageEvent<CustomMsgsSystemUsage>;
          setState((prevState) => ({
            ...prevState,
            cpuUsage: systemUsagelatestFrame.message.cpu_percent,
            ramUsage: systemUsagelatestFrame.message.ram.percentage,
          }));
        } else if (latestFrame.topic === VOLTAGE_TOPIC) {
          const voltageLatestFrame = latestFrame as MessageEvent<StdMsgsFloat64>;
          setState((prevState) => ({
            ...prevState,
            voltage: voltageLatestFrame.message.data,
          }));
        } else if (latestFrame.topic === HUMIDITY_TOPIC) {
          const humidityLatestFrame = latestFrame as MessageEvent<StdMsgsFloat64>;
          setState((prevState) => ({
            ...prevState,
            humidity: humidityLatestFrame.message.data,
          }));
        } else if (latestFrame.topic === TEMPERATURE_TOPIC) {
          const temperatureLatestFrame = latestFrame as MessageEvent<StdMsgsFloat64>;
          setState((prevState) => ({
            ...prevState,
            temperature: temperatureLatestFrame.message.data,
          }));
        }
      }
    };
    context.watch("currentFrame");
    context.watch("didSeek");
  }, [context]);

  // Call our done function at the end of each render
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // Render a table with the current system usage data
  const theme = useTheme();
  return (
    <ThemeProvider theme={theme}>
      <Box m={1}>
        <TableContainer component={Paper}>
          <Table size="small">
            <TableBody>
              {rows.map((row) => (
                <TableRow
                  key={row.statusName}
                  style={{
                    backgroundColor: (() => {
                      if (row.value == undefined) {
                        return theme.palette.error.dark;
                      } else if (row.warn()) {
                        return theme.palette.warning.main;
                      }
                      return theme.palette.success.dark;
                    })(),
                  }}
                >
                  <TableCell>
                    <Typography variant="subtitle2" color={theme.palette.common.white}>
                      {row.statusName}
                    </Typography>
                  </TableCell>
                  <TableCell align="right">
                    <Typography variant="subtitle2" color={theme.palette.common.white}>
                      {row.value?.toFixed(1)}
                      {row.suffix}
                    </Typography>
                  </TableCell>
                </TableRow>
              ))}
            </TableBody>
          </Table>
        </TableContainer>
      </Box>
    </ThemeProvider>
  );
}

export function initSystemStatusPanel(context: PanelExtensionContext): () => void {
  context.panelElement.style.overflow = "auto"; // Enable scrolling

  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<SystemStatusPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
