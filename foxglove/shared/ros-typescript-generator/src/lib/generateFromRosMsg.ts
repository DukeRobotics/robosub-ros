// Refactored this file to be compatible with @foxglove/rosmsg 5.0.2 and our linter

// Use MessageDefinitionField instead of RosMsgField
import { MessageDefinitionField } from '@foxglove/message-definition';
import { parse, fixupTypes } from '@foxglove/rosmsg';
import { ros1 } from '@foxglove/rosmsg-msgs-common';
import { camelCase, compact, partition, upperFirst } from 'lodash';

import { primitives1, primitives2 } from './primitives';
import { IConfig } from '../types/config';

const SUPPORTED_ROS_VERSIONS = [1, 2];

const rosNameToTypeName = (rosName: string, prefix = '') =>
  `${prefix}${upperFirst(camelCase(rosName))}`;

/** Take in a ros definition string and generates a typescript interface */
export const generateFromRosMsg = (
  rosDefinition: string,
  typePrefix = '',
  rosVersion: IConfig['rosVersion'] = 2,
): boolean | string => {
  if (!SUPPORTED_ROS_VERSIONS.includes(rosVersion)) {
    throw new Error('Unsupported rosVersion');
  }

  // Skip type fixup because it will fail as we don't use gentools to include all dependent types
  const messageDefinitions = parse(rosDefinition, {
    ros2: rosVersion === 2,
    skipTypeFixup: true,
  });
  const primitives = rosVersion === 1 ? primitives1 : primitives2;

  // Create a new array including the current defintion and all ros1 message definitions and fixup types on that
  // This modifies the original messageDefinitions array
  const allMessageDefinitions = Object.values(messageDefinitions).concat(
    Object.values(ros1),
  );
  fixupTypes(allMessageDefinitions);

  function isOfNoneEmptyType(field: MessageDefinitionField): boolean {
    if (!(field.isComplex ?? false)) {
      return true;
    }

    // Use allMessageDefinitions instead of messageDefinitions so ros1 message definitions can be found
    const definition = allMessageDefinitions.find((def) => {
      return def.name === field.type;
    });

    if (definition) {
      return definition.definitions.length > 0;
    }

    throw new Error(
      `Field with type "${field.type}" doesn't exist in message definitions`,
    );
  }

  function toEnumValue(field: MessageDefinitionField) {
    if (field.type === 'bool' && typeof field.value === 'boolean') {
      return field.value ? 1 : 0;
    }
    if (
      field.type === 'char' ||
      field.type === 'wchar' ||
      field.type === 'string' ||
      field.type === 'wstring'
    ) {
      return `'${field.value}'`;
    }

    return field.value;
  }

  return messageDefinitions
    .map((definition) => {
      // Get the interface key
      const typeName = rosNameToTypeName(definition.name ?? '', typePrefix);

      // Find the constant and variable definitions
      const [defConstants, defTypes] = partition(
        definition.definitions,
        (field) => field.isConstant,
      );

      // Generate the ts types for the key val items
      const tsTypes = defTypes
        .filter((defType) => isOfNoneEmptyType(defType))
        .map((param) => {
          const paramType: string =
            param.type in primitives
              ? primitives[param.type as keyof typeof primitives]
              : rosNameToTypeName(param.type, typePrefix);

          const arrayMarker = param.isArray ?? false ? '[]' : '';
          return `  ${param.name}: ${paramType}${arrayMarker};`;
        })
        .join('\n');

      const tsEnum = defConstants
        .map((param) => {
          return `  ${param.name} = ${toEnumValue(param)},`;
        })
        .join('\n');

      const tsTypeFinal =
        tsTypes.length > 0 &&
        `export interface ${typeName} {
${tsTypes}
}`;

      const tsEnumFinal =
        tsEnum.length > 0 &&
        `export enum ${typeName}Const {
${tsEnum}
}`;

      return compact([tsTypeFinal, tsEnumFinal]).join('\n\n');
    })
    .filter((item) => item)
    .sort()
    .join('\n\n');
};
