import { ExtensionContext } from "@foxglove/studio";

import { initThrusterSpeedsSubscriber } from "./ThrusterSpeedSubscriber";
import { initThrusterSpeedsPanel } from "./ThrusterSpeedsPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Thruster Speeds", initPanel: initThrusterSpeedsPanel });
  extensionContext.registerPanel({ name: "Thruster Subscriber", initPanel: initThrusterSpeedsSubscriber });
}
