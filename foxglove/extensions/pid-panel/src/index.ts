import { ExtensionContext } from "@foxglove/studio";

import { initPIDPanel } from "./PIDPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "PID", initPanel: initPIDPanel });
}
