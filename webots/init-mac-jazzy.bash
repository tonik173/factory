#!/bin/bash

# use for mac only
# see https://cyberbotics.com/doc/guide/running-extern-robot-controllers?tab-os=macos&tab-language=python

# When use Webots runs on different host, make sure, the shared folder is mounted!

export WEBOTS_HOME="/Applications/Webots.app"
export DYLD_LIBRARY_PATH="$WEBOTS_HOME/lib/controller"
export PYTHONPATH="$WEBOTS_HOME/Contents/lib/controller/python"
export PYTHONIOENCODING="UTF-8"

multipass mount /Users/komax/RosShared noble:/home/kaan/RosShared
multipass info noble
