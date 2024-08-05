import { ExtensionContext } from "@foxglove/extension";

import { initSystemStatusPanel as subscribeTopicPanel } from "./SystemStatusPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "System Status",
    initPanel: subscribeTopicPanel,
  });
}
