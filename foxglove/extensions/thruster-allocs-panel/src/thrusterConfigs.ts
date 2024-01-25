import { readFile, writeFile, mkdir } from "fs/promises";
import { load } from "js-yaml";
import { join } from "path";
import { format, Options } from "prettier";

import { ThrusterAllocs } from "./ThrusterAllocsPanel";

// Constants and export variable names
const CONFIG_FILE_PATH = "../../../onboard/catkin_ws/src/controls/config/";
const THRUSTER_CONFIGS_SAVE_DIR = "dist";
const EXPORT_VAR_NAME_CONFIGS = "allThrusterConfigs";
const EXPORT_VAR_NAME_ORDERS = "allThrusterOrders";

// Prettier options for code formatting
const PRETTIER_OPTS: Options = {
  parser: "babel",
  arrowParens: "always",
  printWidth: 100,
  trailingComma: "all",
  tabWidth: 2,
  semi: true,
};

// Thruster type definition string for TypeScript
const THRUSTER_TYPE_DEFINITION = `type Thruster = {
  name: string,
  type: string,
  pos: number[],
  rpy: number[],
  flipped: boolean,
};\n\n`;

// Enum type for robot names
enum Robot {
  OOGWAY,
  CTHULHU,
}

// Interface representing the Thruster object
interface Thruster {
  name: string;
  type: string;
  pos: number[];
  rpy: number[];
  flipped: boolean;
}

// Interface representing the thrusters configuration of a robot
interface RobotConfig {
  thrusters: Thruster[];
}

// Types for thruster configurations and orders
type ThrusterConfigs = Record<string, Thruster>;
type AllThrusterConfigs = Record<string, ThrusterConfigs>;
type AllThrusterOrders = Record<string, (keyof ThrusterAllocs)[]>;

// Objects to store thruster configurations and orders
const allThrusterConfigs: AllThrusterConfigs = {};
const allThrusterOrders: AllThrusterOrders = {};

// Function to generate thruster configurations from YAML files
async function generateThrusterConfigs(): Promise<void> {
  for (const robot of Object.keys(Robot)
    .filter((key) => isNaN(Number(key)))
    .map((key) => key)) {
    try {
      // Reading and loading YAML configuration file for each robot
      const configData = await readFile(`${CONFIG_FILE_PATH}${robot.toLowerCase()}.yaml`, { encoding: "utf8" });
      const config: RobotConfig = load(configData) as RobotConfig;

      // Mapping thruster configurations and orders
      const thrusterConfigs: ThrusterConfigs = {};
      const thrusterOrder: (keyof ThrusterAllocs)[] = config.thrusters.map((thruster: Thruster) => {
        const camelCaseThruster = thruster.name.replace(/_([a-z])/g, (g: string) => (g[1] ? g[1].toUpperCase() : ""));
        thrusterConfigs[camelCaseThruster as keyof ThrusterAllocs] = thruster;
        return camelCaseThruster;
      }) as (keyof ThrusterAllocs)[];

      // Storing configurations and orders for each robot
      allThrusterConfigs[robot] = thrusterConfigs;
      allThrusterOrders[robot] = thrusterOrder;
    } catch (error: unknown) {
      console.error(`Error processing robot ${robot}: ${(error as Error).message}`);
    }
  }
}

// Function to write thruster configurations to files
async function writeThrusterConfigs(thrusterConfigsSaveDir: string): Promise<void> {
  const libFile = join(thrusterConfigsSaveDir, "index.js");
  const esmFile = join(thrusterConfigsSaveDir, "index.esm.js");
  const tsFile = join(thrusterConfigsSaveDir, "index.d.ts");

  // Generating libraries
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

  // Creating directories and writing files
  await mkdir(thrusterConfigsSaveDir, { recursive: true });
  await writeFile(libFile, libOutput);
  await writeFile(esmFile, esmOutput);
  await writeFile(tsFile, tsOutput);
}

// Function to generate CommonJS library content from thruster configs and orders data
async function generateCjsLibrary(
  configsData: Record<string, ThrusterConfigs>,
  ordersData: Record<string, (keyof ThrusterAllocs)[]>,
  exportVarNameConfigs: string,
  exportVarNameOrders: string,
): Promise<string> {
  let output = `const ${exportVarNameConfigs} = ${JSON.stringify(configsData, null, 2)};\n`;
  output += `const ${exportVarNameOrders} = ${JSON.stringify(ordersData, null, 2)};\n`;
  output += `module.exports = { ${exportVarNameConfigs}, ${exportVarNameOrders} };`;

  // Formatting the output using Prettier
  return await format(output, PRETTIER_OPTS);
}

// Function to generate ES module library content from thruster configs and orders data
async function generateEsmLibrary(
  configsData: Record<string, ThrusterConfigs>,
  ordersData: Record<string, (keyof ThrusterAllocs)[]>,
  exportVarNameConfigs: string,
  exportVarNameOrders: string,
): Promise<string> {
  let output = `const ${exportVarNameConfigs} = ${JSON.stringify(configsData, null, 2)};\n`;
  output += `const ${exportVarNameOrders} = ${JSON.stringify(ordersData, null, 2)};\n`;
  output += `export { ${exportVarNameConfigs}, ${exportVarNameOrders} };\n`;
  output += `export default { ${exportVarNameConfigs}, ${exportVarNameOrders} };`;

  // Formatting the output using Prettier
  return await format(output, PRETTIER_OPTS);
}

// Function to generate TypeScript library content from thruster configs and orders data
async function generateTsLibrary(exportVarNameConfigs: string, exportVarNameOrders: string): Promise<string> {
  let output = THRUSTER_TYPE_DEFINITION;
  output += `declare const ${exportVarNameConfigs}: { [key: string]: { [key: string]: Thruster } };\n`;
  output += `declare const ${exportVarNameOrders}: { [key: string]: string[] };\n\n`;
  output += `export { ${exportVarNameConfigs} };\n`;
  output += `export { ${exportVarNameOrders} };\n\n`;
  output += `declare const _default: { ${exportVarNameConfigs}: { [key: string]: { [key: string]: Thruster } }, ${exportVarNameOrders}: { [key: string]: string[] } };\n`;
  output += `export default _default;\n`;

  // Formatting the output using Prettier
  return await format(output, { ...PRETTIER_OPTS, parser: "typescript" });
}

async function main() {
  await generateThrusterConfigs();
  await writeThrusterConfigs(THRUSTER_CONFIGS_SAVE_DIR);
}

void main();
