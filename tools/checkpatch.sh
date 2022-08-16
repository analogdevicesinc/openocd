#!/bin/sh
#

git format-patch -M --stdout origin/master | tools/scripts/checkpatch.pl --no-signoff -
