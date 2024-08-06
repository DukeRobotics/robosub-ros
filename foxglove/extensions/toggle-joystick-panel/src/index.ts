import { ExtensionContext } from "@foxglove/extension";

import { initToggleJoystickPanel } from "./ToggleJoystickPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Toggle Joystick", initPanel: initToggleJoystickPanel });
}
