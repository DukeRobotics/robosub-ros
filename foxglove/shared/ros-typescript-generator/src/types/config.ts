export type IConfigDirectory = {
  readonly namespace: string;
  readonly path: string;
};

export type IConfig = {
  readonly output: string;
  readonly rosVersion?: 1 | 2;
  readonly input: readonly IConfigDirectory[];
  readonly typePrefix: string;
};
