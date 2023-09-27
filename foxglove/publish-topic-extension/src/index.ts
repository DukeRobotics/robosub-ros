import { ExtensionContext } from "@foxglove/studio";
import { initPublishTopicPanel } from "./PublishTopicPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Custom Publish Topic Extension", initPanel: initPublishTopicPanel });
}
