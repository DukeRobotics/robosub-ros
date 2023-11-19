import { readFile, writeFile } from "fs/promises";
import { load } from "js-yaml";
import { join } from "path";
import { format, Options } from "prettier";

import { ThrusterSpeeds } from "./ThrusterSpeedsPanel";

const CONFIG_FILE_PATH = "../../../onboard/catkin_ws/src/controls/config/";
const THRUSTER_CONFIGS_SAVE_DIR = "dist";

const EXPORT_VAR_NAME_CONFIGS = "allThrusterConfigs";
const EXPORT_VAR_NAME_ORDERS = "allThrusterOrders";

const PRETTIER_OPTS: Options = {
  parser: "babel",
  arrowParens: "always",
  printWidth: 100,
  trailingComma: "all",
  tabWidth: 2,
  semi: true,
};

const THRUSTER_TYPE_DEFINITION = `type Thruster = {
  name: string,
  type: string,
  pos: number[],
  rpy: number[],
  flipped: boolean,
};\n\n`;

enum Robot {
  OOGWAY,
  CTHULHU,
}

interface Thruster {
  name: string;
  type: string;
  pos: number[];
  rpy: number[];
  flipped: boolean;
}

interface RobotConfig {
  thrusters: Thruster[];
}

type ThrusterConfigs = Record<string, Thruster>;
type AllThrusterConfigs = Record<string, ThrusterConfigs>;
type AllThrusterOrders = Record<string, (keyof ThrusterSpeeds)[]>;

const allThrusterConfigs: AllThrusterConfigs = {};
const allThrusterOrders: AllThrusterOrders = {};

async function generateThrusterConfigs(): Promise<void> {
  for (const robot of Object.keys(Robot)
    .filter((key) => isNaN(Number(key)))
    .map((key) => key)) {
    try {
      const configData = await readFile(`${CONFIG_FILE_PATH}${robot.toLowerCase()}.config`, { encoding: "utf8" });
      const config: RobotConfig = load(configData) as RobotConfig;

      const thrusterConfigs: ThrusterConfigs = {};
      const thrusterOrder: (keyof ThrusterSpeeds)[] = config.thrusters.map((thruster: Thruster) => {
        const camelCaseThruster = thruster.name.replace(/_([a-z])/g, (g: string) => (g[1] ? g[1].toUpperCase() : ""));
        thrusterConfigs[camelCaseThruster as keyof ThrusterSpeeds] = thruster;
        return camelCaseThruster;
      }) as (keyof ThrusterSpeeds)[];

      allThrusterConfigs[robot] = thrusterConfigs;
      allThrusterOrders[robot] = thrusterOrder;
    } catch (error: unknown) {
      console.error(`Error processing robot ${robot}: ${(error as Error).message}`);
    }
  }
}

async function writeThrusterConfigs(thrusterConfigsSaveDir: string): Promise<void> {
  const libFile = join(thrusterConfigsSaveDir, "index.js");
  const esmFile = join(thrusterConfigsSaveDir, "index.esm.js");
  const tsFile = join(thrusterConfigsSaveDir, "index.d.ts");

  const libOutput = await generateCjsLibrary(
    allThrusterConfigs,
    allThrusterOrders,
    EXPORT_VAR_NAME_CONFIGS,
    EXPORT_VAR_NAME_ORDERS,
  );

  const esmOutput = await generateEsmLibrary(
    allThrusterConfigs,
    allThrusterOrders,
    EXPORT_VAR_NAME_CONFIGS,
    EXPORT_VAR_NAME_ORDERS,
  );

  const tsOutput = await generateTsLibrary(EXPORT_VAR_NAME_CONFIGS, EXPORT_VAR_NAME_ORDERS);

  await writeFile(libFile, libOutput);
  await writeFile(esmFile, esmOutput);
  await writeFile(tsFile, tsOutput);
}

async function generateCjsLibrary(
  configsData: Record<string, ThrusterConfigs>,
  ordersData: Record<string, (keyof ThrusterSpeeds)[]>,
  exportVarNameConfigs: string,
  exportVarNameOrders: string,
): Promise<string> {
  let output = `const ${exportVarNameConfigs} = ${JSON.stringify(configsData, null, 2)};\n`;
  output += `const ${exportVarNameOrders} = ${JSON.stringify(ordersData, null, 2)};\n`;
  output += `module.exports = { ${exportVarNameConfigs}, ${exportVarNameOrders} };`;

  return await format(output, PRETTIER_OPTS);
}

async function generateEsmLibrary(
  configsData: Record<string, ThrusterConfigs>,
  ordersData: Record<string, (keyof ThrusterSpeeds)[]>,
  exportVarNameConfigs: string,
  exportVarNameOrders: string,
): Promise<string> {
  let output = `const ${exportVarNameConfigs} = ${JSON.stringify(configsData, null, 2)};\n`;
  output += `const ${exportVarNameOrders} = ${JSON.stringify(ordersData, null, 2)};\n`;
  output += `export { ${exportVarNameConfigs}, ${exportVarNameOrders} };\n`;
  output += `export default { ${exportVarNameConfigs}, ${exportVarNameOrders} };`;

  return await format(output, PRETTIER_OPTS);
}

async function generateTsLibrary(exportVarNameConfigs: string, exportVarNameOrders: string): Promise<string> {
  let output = THRUSTER_TYPE_DEFINITION;
  output += `declare const ${exportVarNameConfigs}: { [key: string]: { [key: string]: Thruster } };\n`;
  output += `declare const ${exportVarNameOrders}: { [key: string]: string[] };\n\n`;
  output += `export { ${exportVarNameConfigs} };\n`;
  output += `export { ${exportVarNameOrders} };\n\n`;
  output += `declare const _default: { ${exportVarNameConfigs}: { [key: string]: { [key: string]: Thruster } }, ${exportVarNameOrders}: { [key: string]: string[] } };\n`;
  output += `export default _default;\n`;

  return await format(output, { ...PRETTIER_OPTS, parser: "typescript" });
}

async function main() {
  await generateThrusterConfigs();
  await writeThrusterConfigs(THRUSTER_CONFIGS_SAVE_DIR);
}

void main();
