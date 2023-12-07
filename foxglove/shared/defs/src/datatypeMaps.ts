import { MessageDefinition } from "@foxglove/message-definition";
import { mkdir, writeFile } from "fs/promises";
import { join } from "path";
import { format, Options } from "prettier";

const primitiveToStandardTypeNameMap: { [key: string]: string } = {
  bool: "std_msgs/Bool",
  int8: "std_msgs/Int8",
  uint8: "std_msgs/UInt8",
  int16: "std_msgs/Int16",
  uint16: "std_msgs/UInt16",
  int32: "std_msgs/Int32",
  uint32: "std_msgs/UInt32",
  int64: "std_msgs/Int64",
  uint64: "std_msgs/UInt64",
  float32: "std_msgs/Float32",
  float64: "std_msgs/Float64",
  string: "std_msgs/String",
  time: "std_msgs/Time",
  duration: "std_msgs/Duration",
};

type AllDatatypeMaps = { [key: string]: { [key: string]: [string, MessageDefinition][] } };

const PRETTIER_OPTS: Options = {
  arrowParens: "always",
  printWidth: 100,
  trailingComma: "all",
  tabWidth: 2,
  semi: true,
};

const EXPORT_VAR_NAME = "allDatatypeMaps";
const EXPORT_TYPE_NAME = "AllDatatypeMapsType";

export async function writeAllDatatypeMaps(
  definitionsByGroup: Map<string, Record<string, MessageDefinition>>,
  allDatatypeMapsSaveDir: string,
  relativePathToCustomMsgs: string,
): Promise<[AllDatatypeMaps, string]> {
  const libFile = join(allDatatypeMapsSaveDir, "index.js");
  const esmFile = join(allDatatypeMapsSaveDir, "index.esm.js");
  const tsFile = join(allDatatypeMapsSaveDir, "index.d.ts");

  const getImportGroupNameFromDefinitionName = getImportGroupNameFromDefinitionNameClosure(definitionsByGroup);

  const allDatatypeMaps = generateDatatypeMaps(definitionsByGroup);
  const allDatatypeMapsString = generateAllDatatypeMapsString(allDatatypeMaps, getImportGroupNameFromDefinitionName);

  const libOutput = await generateCjsLibrary(allDatatypeMapsString, relativePathToCustomMsgs);
  const esmOutput = await generateEsmLibrary(allDatatypeMapsString, relativePathToCustomMsgs);
  const tsOutput = await generateTsLibrary(allDatatypeMaps);

  await mkdir(allDatatypeMapsSaveDir, { recursive: true });
  await writeFile(libFile, libOutput);
  await writeFile(esmFile, esmOutput);
  await writeFile(tsFile, tsOutput);

  return [allDatatypeMaps, allDatatypeMapsString];
}

function generateDatatypeMaps(definitionsByGroup: Map<string, Record<string, MessageDefinition>>): AllDatatypeMaps {
  // All datatype maps for all definitions across all groups
  const allDatatypeMaps: AllDatatypeMaps = {};

  // All definitions across all groups
  let allDefinitions: { [key: string]: MessageDefinition } = {};
  for (const [_, definitions] of definitionsByGroup.entries()) {
    allDefinitions = { ...allDefinitions, ...definitions };
  }

  for (const [groupName, definitions] of definitionsByGroup.entries()) {
    // All datatype maps for the current group
    const groupDatatypeMaps: { [key: string]: [string, MessageDefinition][] } = {};

    for (const [name, definition] of Object.entries(definitions)) {
      // Pairs of complex type names and their definitions used by the current definition
      const mapArray: [string, MessageDefinition][] = [];

      const typesSet = new Set<string>();

      for (const definitionField of definition.definitions) {
        if (typesSet.has(definitionField.type)) {
          continue;
        }

        typesSet.add(definitionField.type);

        let standardTypeName: string | undefined;

        if (definitionField.isComplex === true) {
          standardTypeName = definitionField.type;
        } else if (definitionField.type in primitiveToStandardTypeNameMap) {
          standardTypeName = primitiveToStandardTypeNameMap[definitionField.type];
        }

        if (standardTypeName == undefined) {
          throw new Error(`Unknown type: ${definitionField.type}`);
        }

        const definitionFieldDefinition: MessageDefinition | undefined = allDefinitions[standardTypeName];

        if (definitionFieldDefinition == undefined) {
          throw new Error(`Unknown definition: ${standardTypeName}`);
        }

        mapArray.push([standardTypeName, definitionFieldDefinition]);
      }

      // Add the current definition's name and definition
      mapArray.push([name, definition]);

      // Convert the name and definition pairs into a Map, and add it to the group's maps
      groupDatatypeMaps[name] = mapArray;
    }

    // Add the group's maps to all maps
    allDatatypeMaps[groupName] = groupDatatypeMaps;
  }

  // Return a stringified version of all maps that can be added to the generated index.js
  return allDatatypeMaps;
}

function generateAllDatatypeMapsString(
  allDatatypeMaps: AllDatatypeMaps,
  getImportGroupNameFromDefinitionName: (definitionName: string) => string,
): string {
  let output = `const ${EXPORT_VAR_NAME} = {\n`;
  for (const [groupName, groupMaps] of Object.entries(allDatatypeMaps)) {
    output += `  "${groupName}": {\n`;
    for (const [name, map] of Object.entries(groupMaps)) {
      output += `    "${name}": new Map([\n`;
      for (const [type, definition] of map) {
        if (definition.name == undefined) {
          throw new Error(`Definition name is undefined for type: ${type}`);
        }
        const importGroup: string = getImportGroupNameFromDefinitionName(definition.name);
        output += `      ["${type}", ${importGroup}["${definition.name}"]],\n`;
      }
      output += "    ]),\n";
    }
    output += "  },\n";
  }
  output += "};\n";
  return output;
}

function getImportGroupNameFromDefinitionNameClosure(
  definitionsByGroup: Map<string, Record<string, MessageDefinition>>,
) {
  return (definitionName: string): string => {
    for (const [groupName, definitions] of definitionsByGroup.entries()) {
      if (definitionName in definitions) {
        return groupName;
      }
    }
    throw new Error(`Cannot get import group name of type: ${definitionName}`);
  };
}

async function generateCjsLibrary(allDatatypeMapsString: string, relativePathToCustomMsgs: string): Promise<string> {
  let output = `const ros1 = require("@foxglove/rosmsg-msgs-common").ros1;
  const custom_msgs = require("${relativePathToCustomMsgs}").custom_msgs;\n\n`;

  output += allDatatypeMapsString;
  output += `\nmodule.exports.${EXPORT_VAR_NAME} = ${EXPORT_VAR_NAME};`;
  return await format(output, { ...PRETTIER_OPTS, parser: "babel" });
}

async function generateEsmLibrary(allDatatypeMapsString: string, relativePathToCustomMsgs: string): Promise<string> {
  let output = `import { ros1 } from "@foxglove/rosmsg-msgs-common";
  import { custom_msgs } from "${relativePathToCustomMsgs}";\n\n`;

  output += allDatatypeMapsString;
  output += `\nexport { ${EXPORT_VAR_NAME} };\nexport default { ${EXPORT_VAR_NAME} };`;
  return await format(output, { ...PRETTIER_OPTS, parser: "babel" });
}

async function generateTsLibrary(allDatatypeMaps: AllDatatypeMaps): Promise<string> {
  let output = `import { MessageDefinition } from "@foxglove/message-definition";\n\n`;

  for (const [groupName, groupMaps] of Object.entries(allDatatypeMaps)) {
    output += `export type ${groupName}DatatypeMaps = {\n`;
    for (const [name, _] of Object.entries(groupMaps)) {
      output += `  "${name}": Map<string, MessageDefinition>;\n`;
    }
    output += `};\n\n`;
  }

  output += `export type ${EXPORT_TYPE_NAME} = {\n`;
  for (const [groupName, _] of Object.entries(allDatatypeMaps)) {
    output += `  "${groupName}": ${groupName}DatatypeMaps;\n`;
  }
  output += `};\n\n`;

  output += `declare const ${EXPORT_VAR_NAME}: ${EXPORT_TYPE_NAME};\n`;
  output += `export { ${EXPORT_VAR_NAME} };\n`;
  output += `declare const _default: { ${EXPORT_VAR_NAME}: ${EXPORT_TYPE_NAME} };\n`;
  output += `export default _default;\n`;

  return await format(output, { ...PRETTIER_OPTS, parser: "typescript" });
}
