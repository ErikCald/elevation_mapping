#!/usr/bin/env bash

# set -x

##########################################################
### Constants 
##########################################################

PLATFORM="$(uname -m)"
IMAGE_NAME="elevation_mapping_ros2"

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
SCRIPT_NAME=$(basename "$0")
EM_REPO_DIR=$( cd "$SCRIPT_DIR/.." && pwd)
SRC_DIR=$( cd "$EM_REPO_DIR/.." && pwd)
WORKSPACE_DIR=$( cd "$SRC_DIR/.." && pwd)
WORKSPACE_NAME=$(basename "$WORKSPACE_DIR")
DOCKERFILE_DIR="$WORKSPACE_DIR/docker"

EM_REPO_NAME="elevation_mapping_ros2"
KINDR_REPO_NAME="kindr_ros"


# echo "PLATFORM: $PLATFORM"
# echo "IMAGE_NAME: $IMAGE_NAME"
# echo
# echo "SCRIPT_DIR: $SCRIPT_DIR"
# echo "SCRIPT_NAME: $SCRIPT_NAME"
# echo "WORKSPACE_DIR: $WORKSPACE_DIR"
# echo "WORKSPACE_NAME: $WORKSPACE_NAME"
# echo "DOCKERFILE_DIR: $DOCKERFILE_DIR"

##########################################################
### Safety Checks 
##########################################################

if [ ! -d "$DOCKERFILE_DIR" ]; then
    echo "Missing docker dir at $DOCKERFILE_DIR"
    exit 1
fi

if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "Missing workspace dir at $WORKSPACE_DIR"
    exit 2
fi

if [ ! -d "$WORKSPACE_DIR/src/$EM_REPO_NAME" ]; then
    echo "Missing elevation_mapping_ros2 repo at $WORKSPACE_DIR/src/$EM_REPO_NAME"
    exit 3
fi

if [ ! -d "$WORKSPACE_DIR/src/$KINDR_REPO_NAME" ]; then
    echo "Missing kindr ros repo at $WORKSPACE_DIR/src/$KINDR_REPO_NAME"
    exit 4
fi

##########################################################
### Source helpful scripts
##########################################################

if [ ! -f "$DOCKERFILE_DIR/scripts/sourced_scripts/bash_functions.sh" ]; then
    echo "Cannot find bash_functions.sh file"
    exit 5
else
    # shellcheck disable=SC1091
    source "$DOCKERFILE_DIR/scripts/sourced_scripts/bash_functions.sh" || exit 4
fi

##########################################################
### Docker Build
##########################################################

cd "$DOCKERFILE_DIR" || exit 20

# Copy .dockerignore to temporary workspace to only copy package.xml files
REMOVE_EM_DOCKERIGNORE=0
if [ ! -f "$WORKSPACE_DIR/src/$EM_REPO_NAME/.dockerignore"]; then
    cp "$DOCKERFILE_DIR/container_scripts/.dockerignore" "$WORKSPACE_DIR/src/$EM_REPO_NAME/.dockerignore"
    REMOVE_EM_DOCKERIGNORE=1
fi
REMOVE_KINDR_DOCKERIGNORE=0
if [ ! -f "$WORKSPACE_DIR/src/$KINDR_REPO_NAME/.dockerignore"]; then
    cp "$DOCKERFILE_DIR/container_scripts/.dockerignore" "$WORKSPACE_DIR/src/$KINDR_REPO_NAME/.dockerignore"
    REMOVE_KINDR_DOCKERIGNORE=1
fi

echo
if [ "$PLATFORM" == "aarch64" ]; then
    print_info "Selecting jetson jetpack image for base image"
    # initial_image="nvcr.io/nvidia/l4t-jetpack:${L4T_VERSION}" # TODO: Make L4T verison configurable
    initial_image="nvcr.io/nvidia/l4t-jetpack:r36.4.0"
else
    print_info "Selecting ubuntu:22.04 image for base image"
    initial_image="ubuntu:22.04"
fi

print_docker_build "Building Dockerfile.user as user from $initial_image"
docker build -f "$DOCKERFILE_DIR"/Dockerfile.user \
    --network host \
    --build-arg "BASE_IMAGE=$initial_image" \
    -t user:elevation_mapping . \
    || exit 21

print_docker_build "Building Dockerfile.elevation_mapping as $IMAGE_NAME from user"
docker build -f "$DOCKERFILE_DIR"/Dockerfile.elevation_mapping \
    --network host \
    --build-context "em_repo=$WORKSPACE_DIR/src/$EM_REPO_NAME" \
    --build-context "em_repo=$WORKSPACE_DIR/src/$KINDR_REPO_NAME" \
    --build-arg BASE_IMAGE=user:elevation_mapping \
    -t $IMAGE_NAME . \
    || exit 22

# Remove the .dockerignore files
if [ "$REMOVE_EM_DOCKERIGNORE" -eq 1 ]; then
    rm -f "$WORKSPACE_DIR/src/$EM_REPO_NAME/.dockerignore"
fi
if [ "$REMOVE_KINDR_DOCKERIGNORE" -eq 1 ]; then
    rm -f "$WORKSPACE_DIR/src/$KINDR_REPO_NAME/.dockerignore"
fi

print_info "Built image $IMAGE_NAME successfully"