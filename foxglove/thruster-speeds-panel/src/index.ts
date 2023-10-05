import { ExtensionContext } from "@foxglove/studio";
import { initThrusterSpeedsPanel } from "./ThrusterSpeedsPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Thruster Speeds", initPanel: initThrusterSpeedsPanel });
}
