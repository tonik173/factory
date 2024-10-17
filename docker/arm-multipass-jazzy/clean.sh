#!/bin/sh

docker container stop arm-multipass-jazzy
docker container prune -f
docker image prune -f