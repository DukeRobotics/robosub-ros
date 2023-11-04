import { writeGenerateDatatypeMaps } from "./datatypeMaps";
import { writeMessageDefinitions } from "./messageDefinitions";

async function main() {
  await writeMessageDefinitions();
  await writeGenerateDatatypeMaps();
}

void main();
