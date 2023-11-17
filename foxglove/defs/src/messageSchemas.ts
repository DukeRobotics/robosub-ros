import { MessageDefinition } from "@foxglove/message-definition";
import { mkdir, writeFile } from "fs/promises";
import { join } from "path";
import { format, Options } from "prettier";

type AllJsonSchemas = { [key: string]: { [key: string]: Record<string, unknown> } };

const PRETTIER_OPTS: Options = {
  parser: "json",
  printWidth: 100,
  trailingComma: "all",
  tabWidth: 2,
  semi: true,
};

export async function generateAllJsonSchemas(
  definitionsByGroup: Map<string, Record<string, MessageDefinition>>,
  allJsonSchemasSaveDir: string,
): Promise<AllJsonSchemas> {
  const allJsonSchemas: AllJsonSchemas = {};

  for (const [groupName, definitions] of definitionsByGroup.entries()) {
    const groupJsonSchemas: { [key: string]: Record<string, unknown> } = {};

    for (const [name, definition] of Object.entries(definitions)) {
      const jsonSchema = generateJsonSchema(definition);
      groupJsonSchemas[name] = jsonSchema;
    }

    allJsonSchemas[groupName] = groupJsonSchemas;
  }

  const jsonString = generateAllJsonSchemasString(allJsonSchemas);
  const jsonFilePath = join(allJsonSchemasSaveDir, "allJsonSchemas.json");

  await writeFile(jsonFilePath, jsonString);

  return allJsonSchemas;
}

function generateJsonSchema(definition: MessageDefinition): Record<string, unknown> {
  const properties: Record<string, unknown> = {};

  for (const definitionField of definition.definitions) {
    let fieldType: Record<string, unknown>;

    if (definitionField.isComplex === true) {
      // Recursively handle complex types
      fieldType = generateJsonSchema({
        name: definitionField.type,
        definitions: definitionField.definitions,
      });
    
    switch (definitionField.type) {
      case "string":
        fieldType = { type: "string" };
        break;
      case "boolean":
        fieldType = { type: "boolean" };
        break;
      case "float64":
        fieldType = { type: "number" };
        break;
      case "uint32":
        fieldType = { type: "integer", minimum: 0 };
        break;
      case "time":
        fieldType = {
          type: "object",
          title: "time",
          properties: {
            sec: { type: "integer", minimum: 0 },
            nsec: { type: "integer", minimum: 0, maximum: 999_999_999 },
          },
        };
        break;
      case "duration":
        fieldType = {
          type: "object",
          title: "duration",
          properties: {
            sec: { type: "integer" },
            nsec: { type: "integer", minimum: 0, maximum: 999_999_999 },
          },
        };
        break;
      // Handle other cases or custom types as needed
      default:
        if (definitionField.isComplex) {
          // Recursively generate schema for nested type
          fieldType = generateJsonSchema({
            name: definitionField.type,
            definitions: definitionField?.definitions,
          });
        } else {
          throw new Error(`Unsupported type: ${definitionField.type}`);
        }
    }

    if (definitionField.isArray ?? false) {
      fieldType = {
        type: "array",
        items: fieldType,
      };
    }

    properties[definitionField.name] = fieldType;
  }

  return {
    title: definition.name,
    type: "object",
    properties,
  };
}

function generateAllJsonSchemasString(allJsonSchemas: AllJsonSchemas): string {
  return JSON.stringify(allJsonSchemas, null, 2);
}

async function generateCjsLibrary(allJsonSchemas: AllJsonSchemas): Promise<string> {
  const output = `module.exports = ${generateAllJsonSchemasString(allJsonSchemas)};`;
  return await format(output, PRETTIER_OPTS);
}

async function generateEsmLibrary(allJsonSchemas: AllJsonSchemas): Promise<string> {
  const output = `export default ${generateAllJsonSchemasString(allJsonSchemas)};`;
  return await format(output, PRETTIER_OPTS);
}

async function generateTsLibrary(allJsonSchemas: AllJsonSchemas): Promise<string> {
  const output = `export default ${generateAllJsonSchemasString(allJsonSchemas)};`;
  return await format(output, { ...PRETTIER_OPTS, parser: "typescript" });
}

export async function writeAllJsonSchemas(
  definitionsByGroup: Map<string, Record<string, MessageDefinition>>,
  allJsonSchemasSaveDir: string,
): Promise<AllJsonSchemas> {
  const allJsonSchemas: AllJsonSchemas = {};

  for (const [groupName, definitions] of definitionsByGroup.entries()) {
    const groupJsonSchemas: { [key: string]: Record<string, unknown> } = {};

    for (const [name, definition] of Object.entries(definitions)) {
      const jsonSchema = generateJsonSchema(definition);
      groupJsonSchemas[name] = jsonSchema;
    }

    allJsonSchemas[groupName] = groupJsonSchemas;
  }

  const jsonString = generateAllJsonSchemasString(allJsonSchemas);
  const jsonFilePath = join(allJsonSchemasSaveDir, "allJsonSchemas.json");

  await mkdir(allJsonSchemasSaveDir, { recursive: true });
  await writeFile(jsonFilePath, jsonString);

  // Generate and write CommonJS library
  const cjsLibraryString = await generateCjsLibrary(allJsonSchemas);
  const cjsLibraryFilePath = join(allJsonSchemasSaveDir, "index.js");
  await writeFile(cjsLibraryFilePath, cjsLibraryString);

  // Generate and write ESM library
  const esmLibraryString = await generateEsmLibrary(allJsonSchemas);
  const esmLibraryFilePath = join(allJsonSchemasSaveDir, "index.esm.js");
  await writeFile(esmLibraryFilePath, esmLibraryString);

  // Generate and write TypeScript library
  const tsLibraryString = await generateTsLibrary(allJsonSchemas);
  const tsLibraryFilePath = join(allJsonSchemasSaveDir, "index.d.ts");
  await writeFile(tsLibraryFilePath, tsLibraryString);

  return allJsonSchemas;
}
