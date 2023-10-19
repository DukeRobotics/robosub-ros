import { ExtensionContext } from "@foxglove/studio";

import { initSubscribeTopicPanel as subscribeTopicPanel } from "./SubscribeTopicPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Subscribe Topic", initPanel: subscribeTopicPanel });
}
