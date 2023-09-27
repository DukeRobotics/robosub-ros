import { ExtensionContext } from "@foxglove/studio";

import { initExamplePanel } from "./ExamplePanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "Custom Image Extension",
    initPanel: initExamplePanel,
  });
}
