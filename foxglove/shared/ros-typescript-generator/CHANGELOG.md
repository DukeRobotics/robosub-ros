# Changelog

All notable changes to this project will be documented in this file. See [standard-version](https://github.com/conventional-changelog/standard-version) for commit guidelines.

## [1.7.0](https://github.com/Greenroom-Robotics/ros-typescript-generator/compare/v1.6.4...v1.7.0) (2023-10-10)


### Features

* add support for action and srv files ([#14](https://github.com/Greenroom-Robotics/ros-typescript-generator/issues/14)) ([6baec16](https://github.com/Greenroom-Robotics/ros-typescript-generator/commit/6baec160eb102e03266569d841f55ed263394bad))

### [1.6.4](https://github.com/Greenroom-Robotics/ros-typescript-generator/compare/v1.6.2...v1.6.4) (2023-04-26)


### Bug Fixes

* **#10:** time primitive for ROS1 ([e536709](https://github.com/Greenroom-Robotics/ros-typescript-generator/commit/e536709b922c4ef96de1d777c33961ca189d7b3f)), closes [#10](https://github.com/Greenroom-Robotics/ros-typescript-generator/issues/10)

### [1.6.3](https://github.com/Greenroom-Robotics/ros-typescript-generator/compare/v1.6.2...v1.6.3) (2023-02-10)

### Bug Fixes

* fixed the incorrect typing of `duration` thanks to ([EzraBrooks](https://github.com/Greenroom-Robotics/ros-typescript-generator/pull/9)

### [1.6.2](https://github.com/Greenroom-Robotics/ros-typescript-generator/compare/v1.6.1...v1.6.2) (2022-06-14)


### Bug Fixes

* convert non-number `const` values to valid `enum` values ([20788c5](https://github.com/Greenroom-Robotics/ros-typescript-generator/commit/20788c5f3ee5c67abf7683c041d1d538ad94fc41))

### [1.6.1](https://github.com/Greenroom-Robotics/ros-typescript-generator/compare/v1.6.0...v1.6.1) (2022-05-19)


### Bug Fixes

* don't add fields which are of "empty" type ([04d3b37](https://github.com/Greenroom-Robotics/ros-typescript-generator/commit/04d3b37b791e607d28fd504a74debec093eddd96))
* remove eslint-disable eslint-comments/no-unlimited-disable ([9ab1517](https://github.com/Greenroom-Robotics/ros-typescript-generator/commit/9ab15171374d24ef1e35660e247ec151c250343b))

## [1.6.0](https://github.com/Greenroom-Robotics/ros-typescript-generator/compare/v1.5.0...v1.6.0) (2022-05-06)


### Features

* make ROS version configurable ([e828a00](https://github.com/Greenroom-Robotics/ros-typescript-generator/commit/e828a00506957b0396272694e9102ed068043c25))

## [1.5.0](https://github.com/Greenroom-Robotics/ros-typescript-generator/compare/v1.4.2...v1.5.0) (2022-03-30)


### Features

* add wstring support ([278ee43](https://github.com/Greenroom-Robotics/ros-typescript-generator/commit/278ee433a042491269939aba59bd90a928ab0add))


### Bug Fixes

* spelling ([b090563](https://github.com/Greenroom-Robotics/ros-typescript-generator/commit/b09056365c7d506e43c33f03e93f5789b8eb016e))

### [1.4.2](https://github.com/Greenroom-Robotics/ros-typescript-generator/compare/v1.4.1...v1.4.2) (2022-02-20)

### [1.4.1](https://github.com/Greenroom-Robotics/ros-typescript-generator/compare/v1.4.0...v1.4.1) (2022-02-20)

## [1.4.0](https://github.com/Greenroom-Robotics/ros-typescript-generator/compare/v1.2.0...v1.4.0) (2022-02-20)


### Features

* add real example ([8f7670b](https://github.com/Greenroom-Robotics/ros-typescript-generator/commit/8f7670bfd4b072793c7eb5f7757f2ff0589e8d9d))

## [1.3.0](https://github.com/MrBlenny/ros-typescript-generator/compare/v1.2.0...v1.3.0) (2022-02-20)


### Features

* add real example ([2bb5aae](https://github.com/MrBlenny/ros-typescript-generator/commit/2bb5aae5adb2e3b1d22480138bd8d10cebbad965))

## [1.2.0](https://github.com/MrBlenny/ros-typescript-generator/compare/v1.1.0...v1.2.0) (2022-02-16)


### Features

* add a failing test for enum grouping ([9eaa812](https://github.com/MrBlenny/ros-typescript-generator/commit/9eaa8122978ada002073ef7d6e42b7b1e1c41714))
* add enum support ([595a67b](https://github.com/MrBlenny/ros-typescript-generator/commit/595a67b4ac36e373dd531da39a663947e43cfa89))
* skip enum test ([7b22b5f](https://github.com/MrBlenny/ros-typescript-generator/commit/7b22b5ffc34a0c760422d2d7a7a796f847b32a40))

## 1.1.0 (2022-02-01)


### Features

* add circleci badge ([e0404ca](https://github.com/MrBlenny/ros-typescript-generator/commit/e0404cadb6f388107edee68019e7d818c0ed28fb))
* add logic ([f5e65ae](https://github.com/MrBlenny/ros-typescript-generator/commit/f5e65ae178bea35ee0caecb13c8ce38f335ef8ed))
* add more types ([83d980c](https://github.com/MrBlenny/ros-typescript-generator/commit/83d980c94c2c5e58f05978cc75af1e0a6e2c0f18))
* add test_data ([596f570](https://github.com/MrBlenny/ros-typescript-generator/commit/596f57098b904fe08499722a57d7bf1a6989990c))
* export the gened ros_msgs ([318665d](https://github.com/MrBlenny/ros-typescript-generator/commit/318665d45c3728ae4b38b28cb9dc305543ca5474))
* initialise repo ([6eb194a](https://github.com/MrBlenny/ros-typescript-generator/commit/6eb194a5e1d7ed7d8518c614aeae84ef8be6eb3b))
* working ([be02eca](https://github.com/MrBlenny/ros-typescript-generator/commit/be02ecafb0f9f43faa072d86a93da90295a6cadc))


### Bug Fixes

* bad md autoformat ([2966a0f](https://github.com/MrBlenny/ros-typescript-generator/commit/2966a0f9a4b33a1a6728302fa9651d185066e13e))
* doc link ([ac0a14b](https://github.com/MrBlenny/ros-typescript-generator/commit/ac0a14b0e4605e5ac84c62afde504df76e8f1522))
* don't spellcheck generated ([48a9550](https://github.com/MrBlenny/ros-typescript-generator/commit/48a95506f49475b8fdb160d4a34846fa1a0202cc))
* fix spelling ([dfe0221](https://github.com/MrBlenny/ros-typescript-generator/commit/dfe0221b8c39f71a6ead5c235d9aef8e97f3ae53))
* formatting ([a53726c](https://github.com/MrBlenny/ros-typescript-generator/commit/a53726c71ef6986c29a92a836d4e7672aaea201a))
* time should be sec/nanosec ([655acb3](https://github.com/MrBlenny/ros-typescript-generator/commit/655acb3f2510ff5bcc22d3b6b6c1c3426c3eef34))
