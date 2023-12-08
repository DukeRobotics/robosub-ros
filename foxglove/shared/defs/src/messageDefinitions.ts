import { MessageDefinition } from "@foxglove/message-definition";
import { parse, ParseOptions, fixupTypes } from "@foxglove/rosmsg";
import { ros1 } from "@foxglove/rosmsg-msgs-common";
import { mkdir, readdir, readFile, writeFile } from "fs/promises";
import { join, basename, sep } from "path";
import { format, Options } from "prettier";

const PRETTIER_OPTS: Options = {
  arrowParens: "always",
  printWidth: 100,
  trailingComma: "all",
  tabWidth: 2,
  semi: true,
};

const CUSTOM_MSGS_GROUP_NAME = "custom_msgs";

export async function writeMessageDefinitions(
  customMsgsDefsPath: string,
  customMsgsSaveDir: string,
): Promise<Map<string, Record<string, MessageDefinition>>> {
  // Get paths to save generated message definition files
  const libFile = join(customMsgsSaveDir, "index.js");
  const esmFile = join(customMsgsSaveDir, "index.esm.js");
  const declFile = join(customMsgsSaveDir, "index.d.ts");

  // Map of group to definitions for that group
  // We only have one group, custom_msgs
  const definitionsByGroup = new Map<string, Record<string, MessageDefinition>>([[CUSTOM_MSGS_GROUP_NAME, {}]]);

  // Load all message definitions from msgdefsPath
  // This will populate definitionsByGroup
  // We skip type fixup because we will do that later
  await loadDefinitions(customMsgsDefsPath, definitionsByGroup.get(CUSTOM_MSGS_GROUP_NAME)!, { skipTypeFixup: true });

  // Generate message definition files
  const libOutput = await generateCjsLibrary(definitionsByGroup);
  const esmOutput = await generateEsmLibrary(definitionsByGroup);
  const declOutput = await generateDefinitions(definitionsByGroup);

  // Make sure the save directory exists (if not, create it) and write the files
  await mkdir(customMsgsSaveDir, { recursive: true });
  await writeFile(libFile, libOutput);
  await writeFile(esmFile, esmOutput);
  await writeFile(declFile, declOutput);

  // Return the definitions
  return definitionsByGroup;
}

// Recursively get all .msg files in a directory
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

// Read all message definitions from msgdefsPath, convert them to MessageDefinition objects, and add them to definitions
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
    schema[0]!.name = typeName;
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
  // Included ros1 so that fixupTypes can resolve standard ROS 1 types
  // This allows us to fixup type names in custom msg definitions without having to use gentools
  const allTypes = Object.values(definitions).concat(Object.values(ros1));
  fixupTypes(allTypes);
}

// Extract the data type from a filename
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

// Convert a data type to a type name
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

// Generate a CommonJS library for the message definitions
async function generateCjsLibrary(definitionsByGroup: Map<string, Record<string, MessageDefinition>>): Promise<string> {
  let lib = "";
  for (const [groupName, definitions] of definitionsByGroup.entries()) {
    lib += `
const ${groupName}Definitions = ${JSON.stringify(definitions)}
module.exports.${groupName} = ${groupName}Definitions
`;
  }
  return await format(lib, { ...PRETTIER_OPTS, parser: "babel" });
}

// Generate an ES Module library for the message definitions
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

  return await format(lib, { ...PRETTIER_OPTS, parser: "babel" });
}

// Generate a TypeScript definitions file for the message definitions
async function generateDefinitions(
  definitionsByGroup: Map<string, Record<string, MessageDefinition>>,
): Promise<string> {
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
  return await format(output, { ...PRETTIER_OPTS, parser: "typescript" });
}

function exportedTypeName(groupName: string): string {
  // Uppercase the first letter of `groupName` and any letter following a number
  const camelCase = `${groupName[0]!.toUpperCase()}${groupName
    .slice(1)
    .replace(/([0-9])([a-z])/, (m) => m[0]! + m[1]!.toUpperCase())}`;
  return `${camelCase}MsgCommonDefinitions`;
}
