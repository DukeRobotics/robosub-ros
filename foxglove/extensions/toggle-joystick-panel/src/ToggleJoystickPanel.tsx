import { allDatatypeMaps } from "@duke-robotics/defs/datatype_maps";
import {
  GeometryMsgsTwist,
  CustomMsgsSetControlTypesRequest,
  CustomMsgsSetControlTypesResponse,
  CustomMsgsControlTypesConst,
} from "@duke-robotics/defs/types";
import useTheme from "@duke-robotics/theme";
import { Immutable, PanelExtensionContext, RenderState } from "@foxglove/studio";
import { Button, Box, Alert, ThemeProvider } from "@mui/material";
import { JsonViewer } from "@textea/json-viewer";
import { SetStateAction, useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

const PUBLISH_RATE = 20; // Publish rate in Hz
const DEBUG = true; // Set to true to display live transformed joystick inputs

// Indices of specific axes in the joystick axes list
const AXIS_MAP = {
  xIndex: 1, // Forward and backward, inverted
  yIndex: 0, // Left and right
  zIndex: 2, // Up and down, inverted
  yawIndex: 5, // Twist CW and CCW
  pitchIndex: 9,
  rollIndex: 9,
};

/**
 * Calculate power to publish to /controls/desired_power for pitch from thumb joystick input
 * @param value joystick value for thumb joystick
 * @returns desired power
 */
const pitchMapping = (value: number): number => {
  // Thumb joystick input initialized at 0 but returns to this stationary value after being touched and released
  const stationary = 3.2857141494750977;

  if (value !== stationary && value !== 0) {
    if (value > 0.7142857313156128 || value < -0.4285714030265808) {
      return 0.5;
    } else if (value < 0.7142857313156128 && value > -0.4285714030265808) {
      return -0.5;
    } else {
      return 0;
    }
  } else {
    return 0;
  }
};

/**
 * Calculate power to publish to /controls/desired_power for roll from thumb joystick input
 * @param value joystick value for thumb joystick
 * @returns desired power
 */
const rollMapping = (value: number): number => {
  // Thumb joystick input initialized at 0 but returns to this stationary value after being touched and released
  const stationary = 3.2857141494750977;

  if (value !== stationary && value !== 0) {
    if (value > -1 && value < 0.14285719394683838) {
      return 0.5;
    } else if (value > 0.14285719394683838) {
      return -0.5;
    } else {
      return 0;
    }
  } else {
    return 0;
  }
};

const linearMapping = (value: number, { negate = false }: { negate: boolean }): number => {
  const threshold = 0.01;
  if (Math.abs(value) < threshold) {
    return 0;
  }
  return negate ? -1 * value : value;
};

// Indices of specific buttons in the joystick buttons list
const BUTTON_MAP = {
  torpedoActivateIndex: 1,
  torpedoOneLaunchIndex: 4,
  torpedoTwoLaunchIndex: 5,
};

type TransformedJoystickInputs = {
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
  request?: string;
  schemaName: string;
  error?: Error;
  colorScheme?: RenderState["colorScheme"];
  joyStickEnabled: boolean;
  joystickInputs: TransformedJoystickInputs;
};

const SCHEMA_NAME = "geometry_msgs/Twist";
const DESIRED_POWER_TOPIC = "/controls/desired_power";
const SET_CONTROL_TYPES_SERVICE = "/controls/set_control_types";

function ToggleJoystickPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [state, setState] = useState<State>({
    topicName: "",
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
    context.callService(SET_CONTROL_TYPES_SERVICE, request).then(
      (response) => {
        const typedResponse = response as CustomMsgsSetControlTypesResponse;

        // Update the state based on the service response
        // If the service responds with failure, display the response message as an error
        const error = typedResponse.success ? undefined : Error("/controls/set_control_types has failed");
        setState((oldState) => ({ ...oldState, error, joyStickEnabled: !oldState.joyStickEnabled }));
      },
      (error) => {
        // Handle service call errors (e.g., service is not advertised)
        setState((oldState) => ({
          ...oldState,
          error: error as Error,
        }));
      },
    );
  };

  useEffect(() => {
    // Pubish a request with a given schema to a topic
    const publishSpeeds = () => {
      if (!context.advertise || !context.publish) {
        console.log("return");
        return;
      }

      context.advertise(DESIRED_POWER_TOPIC, SCHEMA_NAME, {
        datatypes: allDatatypeMaps.ros1[SCHEMA_NAME],
      });

      // Create and publish request with values from joystick input stored in the state
      const joystickInputs = state.joystickInputs;
      const request: GeometryMsgsTwist = {
        linear: {
          x: joystickInputs.xAxis,
          y: joystickInputs.yAxis,
          z: joystickInputs.zAxis,
        },
        angular: {
          x: joystickInputs.rollAxis,
          y: joystickInputs.pitchAxis,
          z: joystickInputs.yawAxis,
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

  useEffect(() => {
    // eslint-disable-next-line @typescript-eslint/no-unnecessary-condition
    if (DEBUG) {
      const handleJoystickConnected = () => {
        console.log("Joystick connected");
      };
      const handleJoystickDisconnected = () => {
        console.log("Joystick disconnected");
      };

      // Add event listeners
      window.addEventListener("gamepadconnected", handleJoystickConnected);
      window.addEventListener("gamepaddisconnected", handleJoystickDisconnected);

      // Cleanup function to remove event listeners
      return () => {
        window.removeEventListener("gamepadconnected", handleJoystickConnected);
        window.removeEventListener("gamepaddisconnected", handleJoystickDisconnected);
      };
    }
    return;
  }, []);

  const theme = useTheme();
  return (
    <ThemeProvider theme={theme}>
      <Box m={1}>
        {/* Error messages */}
        {(context.callService == undefined || state.error != undefined) && (
          <Box mb={1}>
            {context.callService == undefined && (
              <Alert variant="filled" severity="error">
                Calling services is not supported by this connection
              </Alert>
            )}
            {state.error != undefined && (
              <Alert variant="filled" severity="error">
                {state.error.message}
              </Alert>
            )}
          </Box>
        )}

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

        {/* Table of joystick inputs */}
        {/* eslint-disable-next-line @typescript-eslint/no-unnecessary-condition */}
        {DEBUG && (
          <JsonViewer
            rootName={false}
            value={state.joystickInputs}
            indentWidth={2}
            theme={state.colorScheme}
            enableClipboard={false}
            displayDataTypes={false}
          />
        )}
      </Box>
    </ThemeProvider>
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

/**
 * Query joystick for current state to read input from joystick
 * @param state state object containing joystick inputs and whether the joystick is enabled
 * @param setState method to update the state
 * @param publishSpeeds method to publish joystick inputs
 */
function queryJoystick(state: State, setState: React.Dispatch<SetStateAction<State>>, publishSpeeds: () => void) {
  const joystick = navigator.getGamepads();
  if (joystick[0]) {
    const axes = joystick[0].axes;
    const buttons = joystick[0].buttons;

    // Update state
    setState((previousState) => ({
      ...previousState,
      joystickInputs: {
        xAxis: linearMapping(axes[AXIS_MAP.xIndex] ?? 0, { negate: true }),
        yAxis: linearMapping(axes[AXIS_MAP.yIndex] ?? 0, { negate: true }),
        zAxis: linearMapping(axes[AXIS_MAP.zIndex] ?? 0, { negate: true }),
        yawAxis: linearMapping(axes[AXIS_MAP.yawIndex] ?? 0, { negate: true }),
        pitchAxis: pitchMapping(axes[AXIS_MAP.pitchIndex] ?? 0),
        rollAxis: rollMapping(axes[AXIS_MAP.rollIndex] ?? 0),
        torpedoActivate: buttons[BUTTON_MAP.torpedoActivateIndex]?.value === 1,
        torpedoOneLaunch: buttons[BUTTON_MAP.torpedoOneLaunchIndex]?.value === 1,
        torpedoTwoLaunch: buttons[BUTTON_MAP.torpedoTwoLaunchIndex]?.value === 1,
      },
    }));
  }

  // Publish
  if (state.joyStickEnabled) {
    publishSpeeds();
  }
}
