import { MessageDefinition } from "@foxglove/message-definition";
import { mkdir, writeFile } from "fs/promises";
import { join } from "path";
import { format, Options } from "prettier";

// Map of TS primitive types to ROS standard types
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

type MapArray = [string, MessageDefinition][];
type GroupDatatypeMaps = { [key: string]: MapArray };
type AllDatatypeMaps = { [key: string]: GroupDatatypeMaps };

const PRETTIER_OPTS: Options = {
  arrowParens: "always",
  printWidth: 100,
  trailingComma: "all",
  tabWidth: 2,
  semi: true,
};

// Name of the variable in the exported dist that will hold all datatype maps and the type of that variable
const EXPORT_VAR_NAME = "allDatatypeMaps";
const EXPORT_TYPE_NAME = "AllDatatypeMapsType";

export async function writeAllDatatypeMaps(
  definitionsByGroup: Map<string, Record<string, MessageDefinition>>,
  allDatatypeMapsSaveDir: string,
  relativePathToCustomMsgs: string,
): Promise<[AllDatatypeMaps, string]> {
  // Get paths to save generated datatype map files
  const libFile = join(allDatatypeMapsSaveDir, "index.js");
  const esmFile = join(allDatatypeMapsSaveDir, "index.esm.js");
  const tsFile = join(allDatatypeMapsSaveDir, "index.d.ts");

  // Generate all datatype maps and a stringified version of them
  const allDatatypeMaps = generateDatatypeMaps(definitionsByGroup);
  const allDatatypeMapsString = generateAllDatatypeMapsString(allDatatypeMaps);

  // Generate the generated index.js, index.esm.js, and index.d.ts
  // The index.js and index.esm.js files export the stringified version of all datatype maps
  // The index.d.ts file exports the types of all datatype maps
  const libOutput = await generateCjsLibrary(allDatatypeMapsString, relativePathToCustomMsgs);
  const esmOutput = await generateEsmLibrary(allDatatypeMapsString, relativePathToCustomMsgs);
  const tsOutput = await generateTsLibrary(allDatatypeMaps);

  // Write the generated files to the filesystem
  await mkdir(allDatatypeMapsSaveDir, { recursive: true });
  await writeFile(libFile, libOutput);
  await writeFile(esmFile, esmOutput);
  await writeFile(tsFile, tsOutput);

  // Return the generated datatype maps and the stringified version of them
  return [allDatatypeMaps, allDatatypeMapsString];
}

// Generate all datatype maps for all definitions across all groups
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
    const groupDatatypeMaps: GroupDatatypeMaps = {};

    for (const [name, definition] of Object.entries(definitions)) {
      // Pairs of complex type names and their definitions used by the current definition
      const mapArray: MapArray = [];

      // Set of type names used by the current definition
      const typesSet = new Set<string>();

      for (const definitionField of definition.definitions) {
        // Get the standard type name for the current definition field
        // If the field is complex, the standard type name is the field's type
        // If the field is primitive, the standard type name is the field's type mapped to a standard type name
        let standardTypeName: string;
        if (definitionField.isComplex === true) {
          standardTypeName = definitionField.type;
        } else {
          standardTypeName = primitiveToStandardTypeNameMap[definitionField.type]!;
        }

        // Ensure that each type name is only added once to mapArray
        if (typesSet.has(standardTypeName)) {
          continue;
        }

        // Get the MessageDefintion for the standard type name
        const definitionFieldDefinition: MessageDefinition = allDefinitions[standardTypeName]!;

        // Add the standard type name and definition for this field to mapArray for this definition
        mapArray.push([standardTypeName, definitionFieldDefinition]);

        // Add the standard type name to the set of type names that have already been added to mapArray for this definition
        typesSet.add(standardTypeName);
      }

      // Add the current definition's name and definition to its own mapArray
      mapArray.push([name, definition]);

      // Add name and definition pairs to the group's maps
      groupDatatypeMaps[name] = mapArray;
    }

    // Add the group's maps to all maps
    allDatatypeMaps[groupName] = groupDatatypeMaps;
  }

  // Return a stringified version of all maps that can be added to the generated index.js
  return allDatatypeMaps;
}

// Generate a stringified version of all datatype maps that can be added to the generated index.js and index.esm.js
// Note: The generated stringified version of all datatype maps is valid JS code. It is not a JSON string.
function generateAllDatatypeMapsString(allDatatypeMaps: AllDatatypeMaps): string {
  let output = `const ${EXPORT_VAR_NAME} = {\n`;
  for (const [groupName, groupMaps] of Object.entries(allDatatypeMaps)) {
    output += `  "${groupName}": {\n`;
    for (const [name, map] of Object.entries(groupMaps)) {
      output += `    "${name}": new Map([\n`;
      for (const [type, definition] of map) {
        if (definition.name == undefined) {
          throw new Error(`Definition name is undefined for type: ${type}`);
        }

        // Find the group that definition.name is in
        let definitionGroup: string | undefined;
        for (const [group, definitions] of Object.entries(allDatatypeMaps)) {
          if (definitions[definition.name] != undefined) {
            definitionGroup = group;
            break;
          }
        }

        if (definitionGroup == undefined) {
          throw new Error(`Definition group is undefined for type: ${type}`);
        }

        output += `      ["${type}", ${definitionGroup}["${definition.name}"]],\n`;
      }
      output += "    ]),\n";
    }
    output += "  },\n";
  }
  output += "};\n";
  return output;
}

// Generate a CommonJS library for all datatype maps
async function generateCjsLibrary(allDatatypeMapsString: string, relativePathToCustomMsgs: string): Promise<string> {
  let output = `const ros1 = require("@foxglove/rosmsg-msgs-common").ros1;
  const custom_msgs = require("${relativePathToCustomMsgs}").custom_msgs;\n\n`;

  output += allDatatypeMapsString;
  output += `\nmodule.exports.${EXPORT_VAR_NAME} = ${EXPORT_VAR_NAME};`;
  return await format(output, { ...PRETTIER_OPTS, parser: "babel" });
}

// Generate an ES Module library for all datatype maps
async function generateEsmLibrary(allDatatypeMapsString: string, relativePathToCustomMsgs: string): Promise<string> {
  let output = `import { ros1 } from "@foxglove/rosmsg-msgs-common";
  import { custom_msgs } from "${relativePathToCustomMsgs}";\n\n`;

  output += allDatatypeMapsString;
  output += `\nexport { ${EXPORT_VAR_NAME} };\nexport default { ${EXPORT_VAR_NAME} };`;
  return await format(output, { ...PRETTIER_OPTS, parser: "babel" });
}

// Generate a TypeScript definitions file for all datatype maps
async function generateTsLibrary(allDatatypeMaps: AllDatatypeMaps): Promise<string> {
  let output = `import { MessageDefinition } from "@foxglove/message-definition";\n\n`;

  // Generate a type for each group's datatype maps
  for (const [groupName, groupMaps] of Object.entries(allDatatypeMaps)) {
    output += `export type ${groupName}DatatypeMaps = {\n`;
    for (const [name, _] of Object.entries(groupMaps)) {
      output += `  "${name}": Map<string, MessageDefinition>;\n`;
    }
    output += `};\n\n`;
  }

  // Generate a type for all datatype maps
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
