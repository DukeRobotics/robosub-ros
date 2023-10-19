import { PanelExtensionContext, RenderState } from "@foxglove/studio";
import { useCallback, useEffect, useLayoutEffect, useState } from "react";
import ReactDOM from "react-dom";
import Alert from '@mui/material/Alert';
import { TextField, Button } from "@mui/material";
import { ros1 } from "@foxglove/rosmsg-msgs-common";

type ThrusterSpeeds = {
  frontLeft: number,
  frontRight: number,
  backLeft: number,
  backRight: number,
  bottomFrontLeft: number,
  bottomFrontRight: number,
  bottomBackLeft: number,
  bottomBackRight: number
}

type State = {
  topicName: string;
  request: string;
  error?: Error | undefined;
  colorScheme?: RenderState["colorScheme"];
  thrusterSpeeds: ThrusterSpeeds;
};

function ThrusterSpeedsPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const thrusters = ["frontLeft", "frontRight", "backLeft", "backRight",
                     "bottomFrontLeft", "bottomFrontRight", "bottomBackLeft", "bottomBackRight"]
  const [state, setState] = useState<State>({
    topicName: "",
    request: "{}",
    thrusterSpeeds: {
      frontLeft: 0,
      frontRight: 0,
      backLeft: 0,
      backRight: 0,
      bottomFrontLeft: 0,
      bottomFrontRight: 0,
      bottomBackLeft: 0,
      bottomBackRight: 0
    }
  });

  useLayoutEffect(() => {
    context.onRender = (renderState: RenderState, done) => {
      setState((oldState) => ({ ...oldState, colorScheme: renderState.colorScheme }));
      setRenderDone(() => done);
    };
  }, [context]);

  context.watch("colorScheme");

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const publishSpeeds = useCallback(
    async () => {
      const topicName = "offboard/thruster_speeds"
      const messageType = "custom_msgs/ThrusterSpeeds"

      const thrustersInOrder = ["bottomFrontLeft", "frontLeft", "frontRight", "bottomFrontRight",
                                "bottomBackLeft", "backLeft", "bottomBackRight", "backRight"]
      // const message = `speeds: ${thrustersInOrder.map((thruster) => state.thrusterSpeeds[thruster])}`
      const message = {
        "header": {
          "seq": 0,
          "stamp": {
            "secs": 0,
            "nsecs": 0
          },
          "frame_id": ""
        },
        "speeds": thrustersInOrder.map((thruster) => state.thrusterSpeeds[thruster])
      }

      console.log(ros1["std_msgs/Header"])
      console.log(ros1["sensor_msgs/Joy"])

      if (!context.advertise) {
        return;
      }
      if (!context.publish) {
        return
      }

      const msg_definition = {
        name: "custom_msgs/ThrusterSpeeds",
        definitions: [
          {
            isArray: false,
            isComplex: true,
            name: "header",
            type: "std_msgs/Header",
          },
          {
            isArray: true,
            arrayLength: 8,
            isComplex: false,
            name: "speeds",
            type: "int8",
          }
        ]
      }

      try {
        context.advertise(`/${topicName}`, messageType, {
          datatypes: new Map([
            ["std_msgs/Header", ros1["std_msgs/Header"]],
            ["std_msgs/Int8", ros1["std_msgs/Int8"]],
            ["custom_msgs/ThrusterSpeeds", msg_definition],
          ]),
        });
        context.publish(`/${topicName}`, message);

        setState((oldState) => ({
          ...oldState,
          error: undefined,
        }));
      } catch (error) {
        setState((oldState) => ({ ...oldState, error: error as Error }));
        console.error(error);
      }
    },
    [context.advertise, context.publish],
  );

  const validateInput = (number) => {
    return (number >= -128 && number <= 127)
  }

  const updateSpeeds = (event) => {
    setState((oldState) => ({
      ...oldState,
      thrusterSpeeds: {
        ...state.thrusterSpeeds,
        [event.target.id]: event.target.value
      }
    }))
  }

  return (
    <div style={{ padding: "1rem" }}>
      <h2>Thruster Speeds</h2>
      {(context.advertise == undefined || context.publish == undefined) && (
        <Alert variant="filled" severity="error">Publishing topics is not supported by this connection</Alert>
      )}

      <h4>Topic Name</h4>
      <div>
      {thrusters.map((thruster) => (
        <TextField
          id={thruster}
          error= {!validateInput(state.thrusterSpeeds[thruster])}
          helperText={!validateInput(state.thrusterSpeeds[thruster]) ?
            "The value must be an int from -128 to 127" : null}
          label={thruster}
          variant="outlined"
          onChange={updateSpeeds}
        />
      ))}
      </div>
      <Button
        variant="outlined"
        color="success"
        onClick={async () => { await publishSpeeds(); }}
        disabled={context.callService == undefined}
      >
        Publish Thruster Speeds
      </Button>
    </div>
  );
}

export function initThrusterSpeedsPanel(context: PanelExtensionContext): () => void {
  ReactDOM.render(<ThrusterSpeedsPanel context={context} />, context.panelElement);

  // Return a function to run when the panel is removed
  return () => {
    ReactDOM.unmountComponentAtNode(context.panelElement);
  };
}
