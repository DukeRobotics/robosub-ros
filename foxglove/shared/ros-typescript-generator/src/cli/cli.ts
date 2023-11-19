import { readFile } from 'fs/promises';

import chalk from 'chalk';
import { Command } from 'commander';
import ora from 'ora';

import { rosTypescriptGenerator } from '../lib/rosTypescriptGenerator';
import { IConfig } from '../types/config';

(async () => {
  const program = new Command();

  program.option(
    '-c, --config <type>',
    'path to the config file',
    'ros-ts-generator-config.json'
  );

  program.parse(process.argv);

  const options = program.opts();

  const configRaw = await readFile(options.config, { encoding: 'utf-8' });
  const config = JSON.parse(configRaw) as IConfig;

  const spinner = ora('Generating typescript interfaces').start();
  try {
    await rosTypescriptGenerator(config);
    spinner.succeed();
    ora(`Writing file to ${config.output}`).succeed();
  } catch (e: unknown) {
    spinner.fail();
    console.error(chalk.red(e));
    process.exit(1);
  }
})();
