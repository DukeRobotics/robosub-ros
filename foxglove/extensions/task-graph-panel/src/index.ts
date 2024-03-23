import { ExtensionContext } from "@foxglove/studio";

import { initTaskGraphPanel as taskGraphPanel } from "./TaskGraphPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Subscribe Topic", initPanel: taskGraphPanel });
}
