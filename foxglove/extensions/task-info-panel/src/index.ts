import { ExtensionContext } from "@foxglove/studio";

import { initTaskInfoPanel as taskInfoPanel } from "./TaskInfoPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Subscribe Topic", initPanel: taskInfoPanel });
}
