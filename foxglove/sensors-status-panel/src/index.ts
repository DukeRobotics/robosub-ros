import { ExtensionContext } from "@foxglove/studio";

import { initSensorsStatusPanel as subscribeTopicPanel } from "./SensorsStatusPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "Sensors Status",
    initPanel: subscribeTopicPanel,
  });
}
