# Duke Robotics Club - RoboSub ROS Docker

![Build Status](https://github.com/dukerobotics/robosub-ros-docker/workflows/build/badge.svg)
![Docker Pulls](https://img.shields.io/docker/pulls/dukerobotics/robosub-ros)

We are a Duke University club competing in the RoboSub Competition. Check out [our website](https://duke-robotics.com).

This repo contains our Dockerfiles and instructions for how to build them, in support of our main [robosub-ros](https://github.com/DukeRobotics/robosub-ros) repo. If you're looking to use these images to run our code, check out the main repo.

Our images are automatically built and pushed to [our Docker Hub repo](https://hub.docker.com/r/dukerobotics/robosub-ros) using GitHub Actions.


## Our images

- `base` - built from Docker's Ubuntu image or NVIDIA's base image to retag for our purposes
- [`core`](core) - built on `base`, never run, contains common components for `onboard` and `landside`
- [`onboard`](onboard) - built on `core`, run on the robot itself
- [`landside`](landside) - built on `core`, run on the landside computer during a pool test


## Building manually

:warning: Our automatic build means that **you should almost never need to do a manual build.** But if you do, read on.

Since we need to build our Dockerfile for multiple platforms (x86 for most computers and ARM for the NVIDIA Jetson TX2), we use the `buildx` command, which allows us to build for multiple platforms simultaneously. [More information on buildx](https://docs.docker.com/buildx/working-with-buildx/).

To use `buildx`, make sure you have experimental features enabled.
- Go to Docker's settings > Command Line > select 'Enable experimental features'.
- Go to Docker's settings > Docker Engine > set 'experimental' to `true`.

You will also need to set-up a builder to do cross-platform builds. Create a new builder to get access to multi-architecture features
```bash
docker buildx create --name multibuilder
```
This will create a new builder with the name 'multibuilder' and 'docker-container' as the driver, giving it more capabilities.

Tell Docker to use that builder
```bash
docker buildx use multibuilder
```

### Building `landside`

See the instructions in the [landside folder](landside/).

### Building `onboard` or `core`

For the `core` and `onboard` images, we generate both arm64 and amd64 images that are then bundled together under a single "primary" tag. So for instance, for onboard, the images are called `onboard-arm64` and `onboard-amd64` while the primary tag is just `onboard`.

:information_source: All of these commands must be executed in the directory containing the Dockerfile.

#### Updating the manifest list
After each build and push of the arm64 or amd64 tags, you should ensure that the images used for the primary tag are up to date. This means pointing the primary tag to the newly built images. In order to do so, you will need to create and push a new manifest list. For example, to do this for the `onboard` image, you would run:

```bash
docker manifest create dukerobotics/robosub-ros:onboard dukerobotics/robosub-ros:onboard-arm64 dukerobotics/robosub-ros:onboard-amd64

docker manifest push -p dukerobotics/robosub-ros:onboard
```

In the event that you get an error about an existing manifest list, check `~/.docker/manifests` to see if there are any existing manifests
that match. If so, delete them and run the commands again.
#### arm64 image
Run the below command to build and push a new ARM image with the `onboard-arm64` tag.

```bash
docker buildx build --platform linux/arm64 -t dukerobotics/robosub-ros:onboard-arm64 --push .
```

Now to update the primary tag, run the command in the section on updating the manifest list.

#### amd64 image
Run the below command to build and push a new x86 image with the `onboard-amd64`:

```bash
docker build --build-arg TARGETPLATFORM=linux/amd64 -t dukerobotics/robosub-ros:onboard-amd64 .

docker push dukerobotics/robosub-ros:onboard-amd64
```

We now have the new image pushed to Docker Hub. Please note that any subsequent commit to master will erase this image.

Now to update the primary tag, run the command in the section on updating the manifest list.

#### Primary tag
The following command will build a new arm64 and amd64 image but **does not** update the `onboard-arm64` and `onboard-amd64` tags. Instead, it will push the new images directly to `onboard`.

This command is therefore not recommended, since it can lead to the images falling out of sync.

```bash
docker buildx build --platform linux/amd64,linux/arm64 -t dukerobotics/robosub-ros:onboard --push .
```

### Building `base`
If you really need to update the base image, the procedure is as follows.

#### `base-arm64`
Because NVIDIA does not tag their images with the correct architecture, you need to create a new Dockerfile that just contains

```dockerfile
FROM nvcr.io/nvidia/l4t-base:r32.3.1
```

Then build it using the buildx commands outlined for the `onboard-arm64` image above.

#### `base-amd64`
This image is just the stock Ubuntu image. To update it, just run:

```bash
docker pull ubuntu:bionic
docker tag ubuntu:bionic dukerobotics/robosub-ros:base-amd64
docker push dukerobotics/robosub-ros:base-amd64
```

### Updating the manifest list for base image
Then update the manifest list at `dukerobotics/robosub-ros:base` to point to the new arm64 and amd64 images.


## Contributing

To contribute to this repository, see [CONTRIBUTING.md](CONTRIBUTING.md).