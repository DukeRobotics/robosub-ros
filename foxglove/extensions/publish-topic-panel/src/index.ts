import { ExtensionContext } from "@foxglove/extension";

import { initPublishTopicPanel } from "./PublishTopicPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Publish Topic", initPanel: initPublishTopicPanel });
}
