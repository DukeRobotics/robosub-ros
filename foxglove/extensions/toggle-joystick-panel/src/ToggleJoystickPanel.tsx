import { ros1 } from "@foxglove/rosmsg-msgs-common";
import { Immutable, PanelExtensionContext, RenderState } from "@foxglove/studio";
import { Table, TableBody, TableCell, TableContainer, TableRow, Paper, Typography, Button, Box } from "@mui/material";
import { SetStateAction, useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

import {
  GeometryMsgsTwist,
  CustomMsgsSetControlTypesRequest,
  CustomMsgsSetControlTypesResponse,
  CustomMsgsControlTypesConst,
} from "./types";

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

type State = {
  topicName: string;
  request: string;
  schemaName: string;
  error?: Error;
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

const SCHEMA_NAME = "geometry_msgs/Twist";
const DESIRED_POWER_TOPIC = "/controls/desired_power";
const SET_CONTROL_TYPES_SERVICE = "/controls/set_control_types";

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

  const toggleJoystick = () => {
    // Check if service calling is supported by the context
    if (!context.callService) {
      console.error("Calling services is not supported by this connection");
      return;
    }

    setState((oldState) => ({ ...oldState, joyStickEnabled: !oldState.joyStickEnabled }));

    // Request payload to toggle controls
    const desiredControl: CustomMsgsControlTypesConst = state.joyStickEnabled
      ? CustomMsgsControlTypesConst.DESIRED_POSE
      : CustomMsgsControlTypesConst.DESIRED_POWER;
    const request: CustomMsgsSetControlTypesRequest = {
      control_types: {
        x: desiredControl,
        y: desiredControl,
        z: desiredControl,
        roll: desiredControl,
        pitch: desiredControl,
        yaw: desiredControl,
      },
    };

    // Make the service call
    // context.callService(SET_CONTROL_TYPES_SERVICE, request).then(
    //   (response) => {
    //     const typedResponse = response as CustomMsgsSetControlTypesResponse;

    //     console.log("toggle joystick response received");

    //     // Update the state based on the service response
    //     // If the service responds with failure, display the response message as an error
    //     const error = typedResponse.success ? undefined : Error("SetControlTypes has failed.");
    //     setState((oldState) => ({ ...oldState, error, joyStickEnabled: !state.joyStickEnabled }));
    //   },
    //   (error) => {
    //     // Handle service call errors (e.g., service is not advertised)
    //     setState((oldState) => ({
    //       ...oldState,
    //       error: error as Error,
    //     }));
    //   },
    // );
  };

  useEffect(() => {
    // Pubish a request with a given schema to a topic
    const publishSpeeds = () => {
      if (!context.advertise || !context.publish) {
        console.log("return");
        return;
      }

      // TODO: Update once foxglove-custom-msgs is merged to master
      context.advertise(DESIRED_POWER_TOPIC, SCHEMA_NAME, {
        datatypes: new Map([
          ["geometry_msgs/Twist", ros1["geometry_msgs/Twist"]],
          ["geometry_msgs/Vector3", ros1["geometry_msgs/Vector3"]],
          ["std_msgs/Float64", ros1["std_msgs/Float64"]],
        ]),
      });

      const joystickInputs = state.joystickInputs;
      const request: GeometryMsgsTwist = {
        linear: {
          x: -joystickInputs.xAxis,
          y: joystickInputs.yAxis,
          z: -joystickInputs.zAxis,
        },
        angular: {
          x: 0,
          y: 0,
          z: -joystickInputs.yawAxis,
        },
      };
      context.publish(DESIRED_POWER_TOPIC, request);
    };

    const intervalDelay = 1000 / PUBLISH_RATE; // Convert Hz to milliseconds
    const intervalId = setInterval(() => {
      queryJoystick(state, setState, publishSpeeds);
    }, intervalDelay);

    return () => {
      clearInterval(intervalId); // Clear the interval on component unmount
    };
  }, [state, setState, context]); // Include dependencies that the effect uses

  window.addEventListener("gamepadconnected", () => {
    console.log("connected");
  });
  window.addEventListener("gamepaddisconnected", () => {
    console.log("disconnected");
  });

  return (
    <div style={{ height: "100%", padding: "1rem" }}>
      {/* Toggle button */}
      <Box m={1}>
        <Button
          variant="contained"
          color={state.joyStickEnabled ? "error" : "success"}
          onClick={toggleJoystick}
          disabled={context.callService == undefined}
        >
          {state.joyStickEnabled ? "Disable Joystick" : "Enable Joystick"}
        </Button>
      </Box>
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

function queryJoystick(state: State, setState: React.Dispatch<SetStateAction<State>>, publishSpeeds: () => void) {
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

  // publish
  if (state.joyStickEnabled) {
    publishSpeeds();
  }
}
