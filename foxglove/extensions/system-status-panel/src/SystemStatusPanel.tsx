import { PanelExtensionContext, RenderState, MessageEvent, Immutable } from "@foxglove/studio";
import {
  Box,
  Typography,
  Paper,
  Table,
  TableBody,
  TableCell,
  tableCellClasses,
  TableContainer,
  TableRow,
  useTheme,
} from "@mui/material";
import { useEffect, useState } from "react";
import { createRoot } from "react-dom/client";

// Topic to watch for system usage data
const SYSTEM_USAGE_TOPIC = "/system/usage";

interface SystemUsage {
  cpu_percent: number;
  cpu_speed: number;
  gpu_percent: number;
  gpu_speed: number;
  gpu_memory: Memory;
  ram: Memory;
  disk: Memory;
}

interface Memory {
  used: number;
  total: number;
  percentage: number;
}

type SystemStatusPanelState = {
  cpuUsage?: number;
  ramUsage?: number;
};

function SystemStatusPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [state, setState] = useState<SystemStatusPanelState>({});

  context.subscribe([{ topic: SYSTEM_USAGE_TOPIC }]);

  // Define values in table
  const rows = [
    { statusName: "CPU", value: state.cpuUsage },
    { statusName: "RAM", value: state.ramUsage },
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
        const latestFrame = renderState.currentFrame.at(-1) as MessageEvent<SystemUsage>;
        setState(() => ({
          cpuUsage: latestFrame.message.cpu_percent,
          ramUsage: latestFrame.message.ram.percentage,
        }));
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
    <Box m={1}>
      <TableContainer component={Paper}>
        <Table
          size="small"
          sx={{
            [`& .${tableCellClasses.root}`]: {
              borderBottom: "none",
            },
          }}
        >
          <TableBody>
            {rows.map((row) => (
              <TableRow
                key={row.statusName}
                style={{
                  backgroundColor:
                    row.value == undefined || row.value >= 90 ? theme.palette.error.main : theme.palette.success.main,
                }}
              >
                <TableCell>
                  <Typography variant="subtitle2" color={theme.palette.common.white}>
                    {row.statusName}
                  </Typography>
                </TableCell>
                <TableCell align="right">
                  <Typography variant="subtitle2" color={theme.palette.common.white}>
                    {row.value?.toFixed(1)}%
                  </Typography>
                </TableCell>
              </TableRow>
            ))}
          </TableBody>
        </Table>
      </TableContainer>
    </Box>
  );
}

export function initSystemStatusPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<SystemStatusPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}