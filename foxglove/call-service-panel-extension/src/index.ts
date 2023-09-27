import { ExtensionContext } from "@foxglove/studio";
import { initCallServicePanel } from "./CallServicePanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Custom Call Service Extension", initPanel: initCallServicePanel });
}
