#!/bin/sh

docker container stop arm-multipass-jazzy-ros
docker container prune -f
docker image prune -f