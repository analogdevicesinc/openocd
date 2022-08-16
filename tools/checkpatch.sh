#!/bin/sh
#

git diff remotes/origin/master | tools/scripts/checkpatch.pl --no-signoff -
