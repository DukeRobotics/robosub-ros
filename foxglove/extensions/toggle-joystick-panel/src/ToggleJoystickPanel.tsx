import { Immutable, PanelExtensionContext, RenderState } from "@foxglove/studio";
import { Table, TableBody, TableCell, TableContainer, TableRow } from "@mui/material";
import Paper from "@mui/material/Paper";
import Alert from "@mui/material/Alert";
import { SetStateAction, useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

const PUBLISH_RATE = 20;
const AXIS_MAP = {
  rightX: 1,
  rightY: 0,
  leftX: 2,
};

type JoystickInputs = {
  xAxis: number;
  yAxis: number;
  zAxis: number;
  YawAxis: number;
  pitchAxis: number;
  rollAxis: number;
  torpedoOne: boolean;
  torpedoTwo: boolean;
};
type State = {
  topicName: string;
  request: string;
  schemaName: string;
  error?: Error | undefined;
  colorScheme?: RenderState["colorScheme"];
  joyStickEnabled: boolean;
  joystickInputs: JoystickInputs;
};

function ToggleJoystickPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const Joystick: (keyof JoystickInputs)[] = [
    "xAxis",
    "yAxis",
    "YawAxis",
    "zAxis",
    "pitchAxis",
    "rollAxis",
    "torpedoOne",
    "torpedoTwo"
  ];
  const [state, setState] = useState<State>({
    topicName: "",
    request: "{}",
    schemaName: "",
    joyStickEnabled: false,
    joystickInputs: {
      xAxis: 0,
      yAxis: 0,
      YawAxis: 0,
      zAxis: 0,
      pitchAxis: 0,
      rollAxis: 0,
      torpedoOne: false,
      torpedoTwo: false,
    },
  });

  const [joystick, setJoystick] = useState<any>([null]);

  // Update color scheme
  useLayoutEffect(() => {
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
  const publishTopic = (topicName: string, request: string, schemaName: string) => {
    if (!context.advertise || !context.publish) {
      return;
    }

    context.advertise(`/${topicName}`, schemaName);
    context.publish(`/${topicName}`, JSON.parse(request));
  };

  // Close publishTopic with the current state for use in the button
  const publishTopicWithRequest = () => {
    publishTopic(state.topicName, state.request, state.schemaName);
  };

  // GAMEPAD STUFF
  window.addEventListener("gamepadconnected", (e) => {
    console.log("connected");
    queryJoystick(setState, setJoystick);
  });

  window.addEventListener("gamepaddisconnected", () => {
    console.log("disconnected");
  });

  return (
    <div style={{ height: "100%", padding: "1rem" }}>
      <TableContainer component={Paper}>
        <Table size="small" aria-label="simple table">
          <TableBody>
            {joystick[0]?.axes.map((axis: number, i: number) => (
              <TableRow
                key={i}
                sx={{ "&:last-child td, &:last-child th": { border: 0 } }}
                style={{ backgroundColor: "white" }}
              >
                <TableCell component="th" scope="row">
                  {axis}
                </TableCell>
              </TableRow>
            ))}
          </TableBody>
        </Table>
      </TableContainer>
    </div>
  );
}

export function initToggleJoystickPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<ToggleJoystickPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}

function queryJoystick(setState: SetStateAction<any>, setJoystick: SetStateAction<any>) {
  const joystick = navigator.getGamepads();
  setJoystick(joystick);

  setTimeout(() => {
    queryJoystick(setState, setJoystick);
  }, 1000 / PUBLISH_RATE);
  // requestAnimationFrame(queryJoystick);
}
