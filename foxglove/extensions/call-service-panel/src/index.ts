import { ExtensionContext } from "@foxglove/extension";

import { initCallServicePanel } from "./CallServicePanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Call Service", initPanel: initCallServicePanel });
}
