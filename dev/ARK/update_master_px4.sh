#!/bin/bash
cd PX4-Autopilot/
git fetch upstream
git checkout upstream/master
git branch -D master
git checkout -b master
git submodule update --init --recursive
git push --set-upstream origin master -f
git remote prune upstream
git remote prune origin
