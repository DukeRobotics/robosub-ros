import { Immutable, PanelExtensionContext, RenderState } from "@foxglove/studio";
import { Table, TableBody, TableCell, TableContainer, TableRow, Paper, Typography } from "@mui/material";
import { SetStateAction, useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

const PUBLISH_RATE = 20;

// indices of specific axes in the joystick axes list
const AXIS_MAP = {
  xIndex: 1, // forward and backward, inverted
  yIndex: 0, // left and right
  zIndex: 2, // up and down, inverted
  yawIndex: 5, // twist CW and CCW
};

// indices of specific buttons in the joystick buttons list
const BUTTON_MAP = {
  torpedoActivateIndex: 1,
  torpedoOneLaunchIndex: 4,
  torpedoTwoLaunchIndex: 5,
};

type JoystickInputs = {
  xAxis: number;
  yAxis: number;
  zAxis: number;
  yawAxis: number;
  pitchAxis: number;
  rollAxis: number;
  torpedoActivate: boolean;
  torpedoOneLaunch: boolean;
  torpedoTwoLaunch: boolean;
};

type Twist = {
  linear: Vector3;
  angular: Vector3;
};

type Vector3 = {
  x: number;
  y: number;
  z: number;
}

type State = {
  topicName: string;
  request: string;
  schemaName: string;
  error?: Error | undefined;
  colorScheme?: RenderState["colorScheme"];
  joyStickEnabled: boolean;
  joystickInputs: JoystickInputs;
};

const JoystickKeys: (keyof JoystickInputs)[] = [
  "xAxis",
  "yAxis",
  "yawAxis",
  "zAxis",
  "pitchAxis",
  "rollAxis",
  "torpedoActivate",
  "torpedoOneLaunch",
  "torpedoTwoLaunch",
];

function ToggleJoystickPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [state, setState] = useState<State>({
    topicName: "",
    request: "{}",
    schemaName: "",
    joyStickEnabled: false,
    joystickInputs: {
      xAxis: 0,
      yAxis: 0,
      yawAxis: 0,
      zAxis: 0,
      pitchAxis: 0,
      rollAxis: 0,
      torpedoActivate: false,
      torpedoOneLaunch: false,
      torpedoTwoLaunch: false,
    },
  });

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

  window.addEventListener("gamepadconnected", () => {
    console.log("connected");
    queryJoystick(state, setState);
  });

  window.addEventListener("gamepaddisconnected", () => {
    console.log("disconnected");
  });

  return (
    <div style={{ height: "100%", padding: "1rem" }}>
      <TableContainer component={Paper}>
        <Table size="small" aria-label="simple table">
          <TableBody>
            {JoystickKeys.map((key: keyof JoystickInputs, i: number) => (
              <TableRow
                key={i}
                sx={{ "&:last-child td, &:last-child th": { border: 0 } }}
                style={{ backgroundColor: "white" }}
              >
                <TableCell>
                  <Typography variant="subtitle2">{key}</Typography>
                </TableCell>
                <TableCell align="right">
                  <Typography variant="subtitle2">{state.joystickInputs[key].toString()}</Typography>
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

function queryJoystick(state: State, setState: React.Dispatch<SetStateAction<State>>) {
  const joystick = navigator.getGamepads();
  if (joystick[0]) {
    const axes = joystick[0].axes;
    const buttons = joystick[0].buttons;

    // update state
    setState((previousState) => ({
      ...previousState,
      joystickInputs: {
        ...previousState.joystickInputs,
        xAxis: axes[AXIS_MAP.xIndex] ?? 0,
        yAxis: axes[AXIS_MAP.yIndex] ?? 0,
        zAxis: axes[AXIS_MAP.zIndex] ?? 0,
        yawAxis: axes[AXIS_MAP.yawIndex] ?? 0,
        pitchAxis: 0, // TODO
        rollAxis: 0, // TODO
        torpedoActivate: buttons[BUTTON_MAP.torpedoActivateIndex]?.value === 1 ? true : false,
        torpedoOneLaunch: buttons[BUTTON_MAP.torpedoOneLaunchIndex]?.value === 1 ? true : false,
        torpedoTwoLaunch: buttons[BUTTON_MAP.torpedoTwoLaunchIndex]?.value === 1 ? true : false,
      },
    }));
  }

  setTimeout(() => {
    queryJoystick(state, setState);
  }, 1000 / PUBLISH_RATE);
}
