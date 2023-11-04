import { MessageDefinition } from "@foxglove/message-definition";
import { parse, ParseOptions, fixupTypes } from "@foxglove/rosmsg";
import { ros1 } from "@foxglove/rosmsg-msgs-common";
import { mkdir, readdir, readFile, writeFile } from "fs/promises";
import { join, basename, sep } from "path";
import { format, Options } from "prettier";

const PRETTIER_OPTS: Options = {
  parser: "babel",
  arrowParens: "always",
  printWidth: 100,
  trailingComma: "all",
  tabWidth: 2,
  semi: true,
};

export async function writeMessageDefinitions(): Promise<Map<string, Record<string, MessageDefinition>>> {
  // Following paths are relative to the foxglove/msgdefs/src directory
  // For the script to work correclty, the script must be run from the foxglove/msgdefs/src directory
  const msgdefsPath = "../../../core/catkin_ws/src/custom_msgs/msg";
  const saveDir = "../custom_msgs";
  const libFile = join(saveDir, "index.js");
  const esmFile = join(saveDir, "index.esm.js");
  const declFile = join(saveDir, "index.d.ts");
  const definitionsByGroup = new Map<string, Record<string, MessageDefinition>>([["custom_msgs", {}]]);

  await loadDefinitions(msgdefsPath, definitionsByGroup.get("custom_msgs")!, { skipTypeFixup: true });

  const libOutput = await generateCjsLibrary(definitionsByGroup);
  const esmOutput = await generateEsmLibrary(definitionsByGroup);
  const declOutput = generateDefinitions(definitionsByGroup);

  await mkdir(saveDir, { recursive: true });
  await writeFile(libFile, libOutput);
  await writeFile(esmFile, esmOutput);
  await writeFile(declFile, declOutput);

  return definitionsByGroup;
}

async function getMsgFiles(dir: string): Promise<string[]> {
  let output: string[] = [];
  for (const entry of await readdir(dir, { withFileTypes: true })) {
    if (entry.isDirectory()) {
      output = output.concat(await getMsgFiles(join(dir, entry.name)));
    } else if (entry.isFile() && entry.name.endsWith(".msg")) {
      output.push(join(dir, entry.name));
    }
  }
  return output;
}

async function loadDefinitions(
  msgdefsPath: string,
  definitions: Record<string, MessageDefinition>,
  parseOptions: ParseOptions,
): Promise<void> {
  const msgFiles = await getMsgFiles(msgdefsPath);
  for (const filename of msgFiles) {
    const dataType = filenameToDataType(filename);
    const typeName = dataTypeToTypeName(dataType);
    const msgdef = await readFile(filename, { encoding: "utf8" });
    const schema = parse(msgdef, parseOptions);
    (schema[0] as MessageDefinition).name = typeName;
    for (const entry of schema) {
      if (entry.name == undefined) {
        throw new Error(`Failed to parse ${dataType} from ${filename}`);
      }
      definitions[entry.name] = entry;
    }
  }
  // Array of MessageDefinitions including all ros1 and custom_msgs
  // This is used to fixup type names in the definitions
  // For example, Header is changed to std_msgs/Header
  // ros1 is included because some custom_msgs use non-primitive ros1 types, and fixupTypes cannot resolve those
  // without including ros1 types
  const allTypes = Object.values(definitions).concat(Object.values(ros1));
  fixupTypes(allTypes);
}

function filenameToDataType(filename: string): string {
  const parts = filename.split(sep);
  const newParts: string[] = [];
  const baseTypeName = basename(parts.pop()!, ".msg");
  while (parts.length > 0) {
    const part = parts.pop()!;
    newParts.unshift(part);
    if (part !== "msg") {
      break;
    }
  }
  return `${newParts.join("/")}/${baseTypeName}`;
}

function dataTypeToTypeName(dataType: string): string {
  const parts = dataType.split("/");
  if (parts.length < 2) {
    throw new Error(`Invalid data type: ${dataType}`);
  }
  const pkg = parts[0]!;
  if (pkg === "msg") {
    throw new Error(`dataType=${dataType}`);
  }
  const name = parts[parts.length - 1]!;
  return `${pkg}/${name}`;
}

async function generateCjsLibrary(definitionsByGroup: Map<string, Record<string, MessageDefinition>>): Promise<string> {
  let lib = "";
  for (const [groupName, definitions] of definitionsByGroup.entries()) {
    lib += `
const ${groupName}Definitions = ${JSON.stringify(definitions)}
module.exports.${groupName} = ${groupName}Definitions
`;
  }
  return await format(lib, PRETTIER_OPTS);
}

async function generateEsmLibrary(definitionsByGroup: Map<string, Record<string, MessageDefinition>>): Promise<string> {
  let lib = "";
  for (const [groupName, definitions] of definitionsByGroup.entries()) {
    lib += `
const ${groupName}Definitions = ${JSON.stringify(definitions)}
export { ${groupName}Definitions as ${groupName} }
`;
  }
  lib += `export default { ${[...definitionsByGroup.keys()]
    .map((groupName) => `${groupName}: ${groupName}Definitions`)
    .join(", ")} }`;

  return await format(lib, PRETTIER_OPTS);
}

function generateDefinitions(definitionsByGroup: Map<string, Record<string, MessageDefinition>>): string {
  let output = `import { MessageDefinition } from "@foxglove/message-definition";`;

  for (const [groupName, definitions] of definitionsByGroup.entries()) {
    const entries = Object.keys(definitions)
      .sort()
      .map((key) => `  "${key}": MessageDefinition;`)
      .join("\n");
    output += `
      export type ${exportedTypeName(groupName)} = {
        ${entries}
      };
    `;
  }

  output += `\n`;
  const groupExportTypes = [...definitionsByGroup.keys()].map(
    (groupName) => `${groupName}: ${exportedTypeName(groupName)}`,
  );

  output += groupExportTypes.map((exportType) => `declare const ${exportType};\n`).join("");
  output += `export { ${[...definitionsByGroup.keys()].join(", ")} };\n`;

  output += `declare const _default: {
  ${groupExportTypes.join(",\n  ")}
}`;
  output += `\nexport default _default;\n`;
  return output
}

function exportedTypeName(groupName: string): string {
  // Uppercase the first letter of `groupName` and any letter following a number
  const camelCase = `${groupName[0]!.toUpperCase()}${groupName
    .slice(1)
    .replace(/([0-9])([a-z])/, (m) => m[0]! + m[1]!.toUpperCase())}`;
  return `${camelCase}MsgCommonDefinitions`;
}
