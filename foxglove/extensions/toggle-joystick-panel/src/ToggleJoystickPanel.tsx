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

const DEBUG = false; // Set to true to display live transformed joystick inputs

const PUBLISH_RATE = 20; // Hz

const SET_CONTROL_TYPES_SERVICE = "/controls/set_control_types";

const DESIRED_POWER_TOPIC = "/controls/desired_power";
const DESIRED_POWER_SCHEMA = "geometry_msgs/Twist";

// Joystick axes indices
const AXIS_MAP = {
  xIndex: 1, // Joystick Forward/Backward
  yIndex: 0, // Joystick Left/Right
  zIndex: 2, // Throttle
  yawIndex: 5, // Joystick Twist
  pitchIndex: 9, // Thumb Joystick Forward/Backward
  rollIndex: 9, // Thumb Joystick Right/Left
};
// Joystick button indices
const BUTTON_MAP = {
  torpedoActivateIndex: 1, // Red/black striped button on Joystick
  torpedoOneLaunchIndex: 4, // Button (5)
  torpedoTwoLaunchIndex: 5, // Button (6)
};

/**
 * Calculate power for pitch from thumb joystick input
 * @param value Value from thumb joystick
 * @returns Desired power
 */
const pitchMapping = (value: number): number => {
  // Thumb joystick input initialized at 0 but returns to this stationary value after being touched and released
  const stationary = 3.2857141494750977;

  if (value !== stationary && value !== 0) {
    if (value > 0.7142857313156128 || value < -0.4285714030265808) {
      return 0.5; // Up
    } else if (value < 0.7142857313156128 && value > -0.4285714030265808) {
      return -0.5; // Down
    } else {
      return 0;
    }
  } else {
    return 0;
  }
};

/**
 * Calculate power for roll from thumb joystick input
 * @param value Value from thumb joystick
 * @returns Desired power
 */
const rollMapping = (value: number): number => {
  // Thumb joystick input initialized at 0 but returns to this stationary value after being touched and released
  const stationary = 3.2857141494750977;

  if (value !== stationary && value !== 0) {
    if (value > -1 && value < 0.14285719394683838) {
      return 0.5; // Right
    } else if (value > 0.14285719394683838) {
      return -0.5; // Left
    } else {
      return 0;
    }
  } else {
    return 0;
  }
};

/**
 * Calculate power for linear axes. If the value is below a threshold, return 0.
 * @param value Value from linear axis
 * @param negate If true, negate the value
 */
const linearMapping = (value: number, { negate = false }: { negate: boolean }): number => {
  const threshold = 0.01;
  if (Math.abs(value) < threshold) {
    return 0;
  }
  return negate ? -value : value;
};

/**
 * Query joystick to read and transform inputs
 * @param state state object containing transformed joystick inputs and whether the joystick is enabled
 * @param setState method to update the state
 * @param publishPower method to publish joystick inputs as a desired power
 */
function queryJoystick(state: State, setState: React.Dispatch<SetStateAction<State>>, publishPower: () => void) {
  const joystick = navigator.getGamepads();
  if (joystick[0]) {
    const axes = joystick[0].axes;
    const buttons = joystick[0].buttons;

    // Update state
    setState((previousState) => ({
      ...previousState,
      transformedJoystickInputs: {
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
    publishPower();
  }
}

type TransformedJoystickInputs = {
  xAxis: number;
  yAxis: number;
  zAxis: number;
  rollAxis: number;
  pitchAxis: number;
  yawAxis: number;
  torpedoActivate: boolean;
  torpedoOneLaunch: boolean;
  torpedoTwoLaunch: boolean;
};

type State = {
  error?: Error;
  colorScheme?: RenderState["colorScheme"];
  joyStickEnabled: boolean;
  transformedJoystickInputs: TransformedJoystickInputs;
};

function ToggleJoystickPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [state, setState] = useState<State>({
    joyStickEnabled: false,
    transformedJoystickInputs: {
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

    // Request payload to toggle control types
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
        if (typedResponse.success) {
          setState((oldState) => ({ ...oldState, error: undefined, joyStickEnabled: !oldState.joyStickEnabled }));
        } else {
          setState((oldState) => ({ ...oldState, error: Error(typedResponse.message) }));
        }
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
    const publishPower = () => {
      if (!context.advertise || !context.publish) {
        return;
      }

      context.advertise(DESIRED_POWER_TOPIC, DESIRED_POWER_SCHEMA, {
        datatypes: allDatatypeMaps.ros1[DESIRED_POWER_SCHEMA],
      });

      // Create and publish desired power message
      const request: GeometryMsgsTwist = {
        linear: {
          x: state.transformedJoystickInputs.xAxis,
          y: state.transformedJoystickInputs.yAxis,
          z: state.transformedJoystickInputs.zAxis,
        },
        angular: {
          x: state.transformedJoystickInputs.rollAxis,
          y: state.transformedJoystickInputs.pitchAxis,
          z: state.transformedJoystickInputs.yawAxis,
        },
      };
      context.publish(DESIRED_POWER_TOPIC, request);
    };

    const intervalDelay = 1000 / PUBLISH_RATE; // Convert Hz to milliseconds
    const intervalId = setInterval(() => {
      queryJoystick(state, setState, publishPower);
    }, intervalDelay);

    return () => {
      clearInterval(intervalId); // Clear the interval on component unmount
    };
  }, [state, setState, context]);

  useEffect(() => {
    // eslint-disable-next-line @typescript-eslint/no-unnecessary-condition
    if (DEBUG) {
      const handleJoystickConnected = () => {
        console.log("Joystick connected");
      };
      const handleJoystickDisconnected = () => {
        console.log("Joystick disconnected");

        setState((oldState) => ({
          ...oldState,
          joyStickEnabled: false,
        }));
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
            value={state.transformedJoystickInputs}
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
