import { ExtensionContext } from "@foxglove/studio";

import { initToggleJoystickPanel } from "./ToggleJoystickPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Toggle Joystick", initPanel: initToggleJoystickPanel });
}
