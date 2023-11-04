import { PanelExtensionContext, RenderState, MessageEvent, Immutable } from "@foxglove/studio";
import { Box, Typography } from "@mui/material";
import Paper from "@mui/material/Paper";
import Table from "@mui/material/Table";
import TableBody from "@mui/material/TableBody";
import TableCell, { tableCellClasses } from "@mui/material/TableCell";
import TableContainer from "@mui/material/TableContainer";
import TableRow from "@mui/material/TableRow";
import { useTheme } from "@mui/material/styles";
import { useLayoutEffect, useEffect, useState } from "react";
import { createRoot } from "react-dom/client";

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
  used: number; // In GB
  total: number; // In GB
  percentage: number; // As a percentage value, e.g., 50 for 50%
}

function createData(status: string, value?: number) {
  return { Status: status, Value: value };
}

type State = {
  cpuUsage?: number;
  ramUsage?: number;
};

function SystemStatusPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  context.subscribe([{ topic: SYSTEM_USAGE_TOPIC }]);

  // Restore our state from the layout via the context.initialState property.
  const [state, setState] = useState<State>({});

  // Define values in table
  const rows = [createData("CPU", state.cpuUsage), createData("RAM", state.ramUsage)];

  // Setup our onRender function and start watching topics and currentFrame for messages.
  useLayoutEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setRenderDone(() => done);

      // Reset state when the user seeks the video
      if (renderState.didSeek ?? false) {
        setState({});
      }

      // Updating CPU/RAM/Voltage data.
      if (renderState.currentFrame && renderState.currentFrame.length > 0) {
        const latestFrame = renderState.currentFrame[renderState.currentFrame.length - 1] as MessageEvent<SystemUsage>;
        setState((oldState) => ({
          ...oldState,
          cpuUsage: latestFrame.message.cpu_percent,
          ramUsage: latestFrame.message.ram.percentage,
        }));
      }
    };

    context.watch("topics");
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
                key={row.Status}
                style={{
                  backgroundColor: (row.Value ?? 100) >= 90 ? theme.palette.error.main : theme.palette.success.main,
                }}
              >
                <TableCell component="th" scope="row">
                  <Typography variant="subtitle2" color={theme.palette.common.white}>
                    {row.Status}
                  </Typography>
                </TableCell>
                <TableCell align="right">
                  <Typography variant="subtitle2" color={theme.palette.common.white}>
                    {row.Value}%
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
