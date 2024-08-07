import { mkdtemp, writeFile } from 'fs/promises';
import { flatten, uniqBy } from 'lodash';
import { tmpdir } from 'os';
import { join } from 'path';

import { generateFromRosMsg } from './generateFromRosMsg';
import { getMsgFilesData } from './readMsgFiles';
import { IConfig } from '../types/config';

export const rosTypescriptGenerator = async (
  config: IConfig,
): Promise<boolean | string> => {
  // TMP dir for generated msg files from action and srv files
  const tempDir = await mkdtemp(join(tmpdir(), 'ros-typescript-generator-'));

  const files = flatten(
    await Promise.all(
      config.input.map(
        async (dir) => await getMsgFilesData(dir.path, dir.namespace, tempDir),
      ),
    ),
  );
  // Remove duplicates
  // Duplicates will occur if the .srv or .action files have already been built
  const filesUnique = uniqBy(files, (file) => file.name + file.namespace);
  const joinedMessages = filesUnique
    .map((file) =>
      [`MSG: ${file.namespace}/${file.name}`, file.data].join('\n'),
    )
    .join('\n===\n');

  const typescriptInterfaces = generateFromRosMsg(
    joinedMessages,
    config.typePrefix,
    config.rosVersion,
  );

  const typescriptInterfacesWithNote = [
    `/* eslint-disable */`,
    `// These files were generated using "ros-typescript-generator"`,
    typescriptInterfaces,
  ].join('\n');

  await writeFile(config.output, typescriptInterfacesWithNote);

  return typescriptInterfaces;
};
