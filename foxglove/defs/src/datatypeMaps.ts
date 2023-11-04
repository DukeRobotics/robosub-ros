import { MessageDefinition } from "@foxglove/message-definition";
import { ros1 } from "@foxglove/rosmsg-msgs-common";

export async function writeGenerateDatatypeMaps(): Promise<void> {
  // test
}

function generateDatatypeMaps(definitionsByGroup: Map<string, Record<string, MessageDefinition>>): string {
  // All datatype maps for all definitions across all groups
  const allMaps: { [key: string]: { [key: string]: [string, MessageDefinition][] } } = {};

  // All definitions across all groups
  let allDefinitions: { [key: string]: MessageDefinition } = {};
  for (const [_, definitions] of definitionsByGroup.entries()) {
    allDefinitions = { ...allDefinitions, ...definitions };
  }
  allDefinitions = { ...allDefinitions, ...ros1 };

  for (const [groupName, definitions] of definitionsByGroup.entries()) {
    // All datatype maps for the current group
    const groupMaps: { [key: string]: [string, MessageDefinition][] } = {};

    for (const [name, definition] of Object.entries(definitions)) {
      // Pairs of complex type names and their definitions used by the current definition
      const mapArray: [string, MessageDefinition][] = [];

      for (const definitionField of definition.definitions) {
        if (definitionField.isComplex === true) {
          mapArray.push([definitionField.type, allDefinitions[definitionField.type]!]);
        }
      }

      // Add the current definition's name and definition
      mapArray.push([name, definition]);

      // Convert the name and definition pairs into a Map, and add it to the group's maps
      groupMaps[name] = mapArray;
    }

    // Add the group's maps to all maps
    allMaps[groupName] = groupMaps;
  }

  // Return a stringified version of all maps that can be added to the generated index.js
  return JSON.stringify(allMaps, null, 2);
}