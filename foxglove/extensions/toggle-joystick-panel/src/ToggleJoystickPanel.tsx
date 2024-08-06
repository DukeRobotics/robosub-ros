import { allDatatypeMaps } from "@duke-robotics/defs/datatype_maps";
import {
  GeometryMsgsTwist,
  CustomMsgsSetControlTypesRequest,
  CustomMsgsSetControlTypesResponse,
  CustomMsgsControlTypesConst,
} from "@duke-robotics/defs/types";
import useTheme from "@duke-robotics/theme";
import { Immutable, PanelExtensionContext, RenderState } from "@foxglove/extension";
import { Button, Box, Alert, ThemeProvider } from "@mui/material";
import { JsonViewer } from "@textea/json-viewer";
import { useCallback, useEffect, useState } from "react";
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
 * @param input Value from linear axis
 * @param gain The value to multiply the input by
 */
const linearMapping = (input: number, gain: number): number => {
  const threshold = 0.01;
  if (Math.abs(input) < threshold) {
    return 0;
  }
  return gain * input;
};

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

type ToggleJoystickPanelState = {
  error?: Error;
  colorScheme?: RenderState["colorScheme"];
  joystickEnabled: boolean;
  transformedJoystickInputs: TransformedJoystickInputs;
  joystickConnected: boolean;
};

function ToggleJoystickPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [state, setState] = useState<ToggleJoystickPanelState>({
    joystickEnabled: false,
    joystickConnected: false,
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
  useEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setState((prevState) => ({ ...prevState, colorScheme: renderState.colorScheme }));
      setRenderDone(() => done);
    };
  }, [context]);
  context.watch("colorScheme");

  // Call our done function at the end of each render
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const toggleJoystick = useCallback(() => {
    // Check if service calling is supported by the context
    if (!context.callService) {
      console.error("Calling services is not supported by this connection");
      return;
    }

    // Request payload to toggle control types
    const desiredControl: CustomMsgsControlTypesConst = state.joystickEnabled
      ? CustomMsgsControlTypesConst.DESIRED_POSITION
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
          setState((prevState) => ({ ...prevState, error: undefined, joystickEnabled: !prevState.joystickEnabled }));
        } else {
          setState((prevState) => ({ ...prevState, error: Error(typedResponse.message) }));
        }
      },
      (error) => {
        // Handle service call errors (e.g., service is not advertised)
        setState((prevState) => ({ ...prevState, error: error as Error }));
      },
    );
  }, [context, state.joystickEnabled]);

  /**
   * Publish transformed joystick inputs as a desired power message
   * @param transformedJoystickInputs A TransformedJoystickInputs object used to create the desired power message
   */
  const publishPower = useCallback(
    (transformedJoystickInputs: TransformedJoystickInputs) => {
      if (!context.advertise || !context.publish) {
        return;
      }

      context.advertise(DESIRED_POWER_TOPIC, DESIRED_POWER_SCHEMA, {
        datatypes: allDatatypeMaps.ros1[DESIRED_POWER_SCHEMA],
      });

      // Create and publish desired power message
      const request: GeometryMsgsTwist = {
        linear: {
          x: transformedJoystickInputs.xAxis,
          y: transformedJoystickInputs.yAxis,
          z: transformedJoystickInputs.zAxis,
        },
        angular: {
          x: transformedJoystickInputs.rollAxis,
          y: transformedJoystickInputs.pitchAxis,
          z: transformedJoystickInputs.yawAxis,
        },
      };
      context.publish(DESIRED_POWER_TOPIC, request);
    },
    [context],
  );

  useEffect(() => {
    /**
     * Query joystick to read and transform inputs
     */
    function queryJoystick() {
      const joystick = navigator.getGamepads();

      if (joystick[0]) {
        const axes = joystick[0].axes;
        const buttons = joystick[0].buttons;

        const transformedJoystickInputs = {
          xAxis: linearMapping(axes[AXIS_MAP.xIndex] ?? 0, -1),
          yAxis: linearMapping(axes[AXIS_MAP.yIndex] ?? 0, -1),
          zAxis: linearMapping(axes[AXIS_MAP.zIndex] ?? 0, 1),
          yawAxis: linearMapping(axes[AXIS_MAP.yawIndex] ?? 0, -0.5),
          pitchAxis: pitchMapping(axes[AXIS_MAP.pitchIndex] ?? 0),
          rollAxis: rollMapping(axes[AXIS_MAP.rollIndex] ?? 0),
          torpedoActivate: buttons[BUTTON_MAP.torpedoActivateIndex]?.value === 1,
          torpedoOneLaunch: buttons[BUTTON_MAP.torpedoOneLaunchIndex]?.value === 1,
          torpedoTwoLaunch: buttons[BUTTON_MAP.torpedoTwoLaunchIndex]?.value === 1,
        };

        // Update state
        setState((prevState) => ({ ...prevState, transformedJoystickInputs }));

        // Publish
        if (state.joystickEnabled) {
          publishPower(transformedJoystickInputs);
          // TODO: Once torpedoes are implemented on Oogway, call the appropriate service to fire torpedoes here
        }
      }
    }

    const intervalDelay = 1000 / PUBLISH_RATE; // Convert Hz to milliseconds
    const intervalId = setInterval(() => {
      queryJoystick();
    }, intervalDelay);

    return () => {
      clearInterval(intervalId); // Clear the interval on component unmount
    };
  }, [publishPower, state.joystickEnabled]);

  useEffect(() => {
    const handleJoystickConnected = () => {
      console.log("Joystick connected");

      setState((prevState) => ({ ...prevState, joystickConnected: true }));
    };
    const handleJoystickDisconnected = () => {
      console.log("Joystick disconnected");

      setState((prevState) => ({ ...prevState, joystickConnected: false }));

      // If the joystick is physically disconnected while enabled, disable it
      // This will ensure that we stop continuously publishing the last known joystick inputs
      if (state.joystickEnabled) {
        toggleJoystick();
      }
    };

    // Add event listeners
    window.addEventListener("gamepadconnected", handleJoystickConnected);
    window.addEventListener("gamepaddisconnected", handleJoystickDisconnected);

    // Cleanup function to remove event listeners
    return () => {
      window.removeEventListener("gamepadconnected", handleJoystickConnected);
      window.removeEventListener("gamepaddisconnected", handleJoystickDisconnected);
    };
  }, [state.joystickEnabled, toggleJoystick]);

  const theme = useTheme();
  return (
    <ThemeProvider theme={theme}>
      <Box m={1}>
        {/* Error messages */}
        {(context.callService == undefined || state.error != undefined) && (
          <Box mb={1}>
            {context.callService == undefined && (
              <Alert variant="filled" severity="error">
                Calling services is not supported by this connection.
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
        <Box my={1}>
          <Button
            fullWidth
            variant="contained"
            color={state.joystickEnabled ? "error" : "success"}
            onClick={toggleJoystick}
            disabled={context.callService == undefined || !state.joystickConnected}
          >
            {state.joystickEnabled ? "Disable Joystick" : "Enable Joystick"}
          </Button>
        </Box>

        {/* View live transformed joystick inputs */}
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
  context.panelElement.style.overflow = "auto"; // Enable scrolling

  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<ToggleJoystickPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
