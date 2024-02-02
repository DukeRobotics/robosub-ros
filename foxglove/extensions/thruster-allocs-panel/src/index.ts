import { ExtensionContext } from "@foxglove/studio";

import { initThrusterAllocsPanel } from "./ThrusterAllocsPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Thruster Allocs", initPanel: initThrusterAllocsPanel });
}
