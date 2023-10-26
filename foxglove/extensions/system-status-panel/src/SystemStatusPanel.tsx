import { PanelExtensionContext, RenderState, MessageEvent, Immutable } from "@foxglove/studio";
import Paper from "@mui/material/Paper";
import Table from "@mui/material/Table";
import TableBody from "@mui/material/TableBody";
import TableCell from "@mui/material/TableCell";
import TableContainer from "@mui/material/TableContainer";
import TableRow from "@mui/material/TableRow";
import { useTheme } from "@mui/material/styles";
import { useLayoutEffect, useEffect, useState } from "react";
import * as React from "react";
import { createRoot } from "react-dom/client";

function createData(Status: string, Value: number) {
  return { Status, Value };
}

type State = {
  topic: string;
  colorScheme?: RenderState["colorScheme"];
  cpuUsage: number;
  ramUsage: number;
};

function SystemStatusPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  const theme = useTheme();

  // Restore our state from the layout via the context.initialState property.
  const [state, setState] = useState<State>(() => {
    const initialState = context.initialState as State;
    initialState.topic = "/system/usage";
    return initialState;
  });

  //define values in table
  const rows = [createData("CPU", state.cpuUsage), createData("RAM", state.ramUsage)];

  useEffect(() => {
    // Save our state to the layout when the topic changes.
    context.saveState({ topic: state.topic });

    if (state.topic) {
      // Subscribe to the new image topic when a new topic is chosen.
      context.subscribe([{ topic: state.topic }]);
    }
  }, [context, state.topic]);

  // Setup our onRender function and start watching topics and currentFrame for messages.
  useLayoutEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setRenderDone(() => done);
      setState((oldState) => ({ ...oldState, colorScheme: renderState.colorScheme }));

      // Updating CPU/RAM/Voltage data.
      if (renderState.currentFrame && renderState.currentFrame.length > 0) {
        const latestFrame = renderState.currentFrame[renderState.currentFrame.length - 1] as MessageEvent<any>;
        setState((oldState) => ({
          ...oldState,
          cpuUsage: latestFrame?.message?.cpu_percent,
          ramUsage: latestFrame?.message?.ram.percentage,
        }));
        console.log(latestFrame?.message?.["cpu_percent"]);
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
      <div>
        <TableContainer component={Paper}>
          <Table size="small" aria-label="simple table">
            <TableBody>
              {rows.map((row) => (
                <TableRow
                  key={row.Status}
                  sx={{ "&:last-child td, &:last-child th": { border: 0 } }}
                  style={{ backgroundColor: row.Value >= 90 ? theme.palette.error.main : "white" }}
                >
                  <TableCell component="th" scope="row">
                    {row.Status}
                  </TableCell>
                  <TableCell align="right">{row.Value}%</TableCell>
                </TableRow>
              ))}
            </TableBody>
          </Table>
        </TableContainer>
      </div>
    </div>
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
