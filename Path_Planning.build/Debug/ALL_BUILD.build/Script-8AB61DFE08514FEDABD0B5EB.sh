#!/bin/sh
make -C /Users/gleb/dev/Udacity-Self-Driving-Car-Engineer/CarND-Path-Planning-Project -f /Users/gleb/dev/Udacity-Self-Driving-Car-Engineer/CarND-Path-Planning-Project/CMakeScripts/ALL_BUILD_cmakeRulesBuildPhase.make$CONFIGURATION OBJDIR=$(basename "$OBJECT_FILE_DIR_normal") all
