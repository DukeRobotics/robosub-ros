import { ExtensionContext } from "@foxglove/studio";
import { initThrusterSpeedsPanel } from "./ThrusterSpeedsPanel";
import { initThrusterSpeedsSubscriber } from "./ThrusterSpeedSubscriber";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Thruster Speeds", initPanel: initThrusterSpeedsPanel });
  extensionContext.registerPanel({ name: "Thruster Subscriber", initPanel: initThrusterSpeedsSubscriber });
}
