import { ExtensionContext } from "@foxglove/studio";

import { initTaskGraphPanel as taskTreePanel } from "./TaskTreePanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Task Tree", initPanel: taskTreePanel });
}
