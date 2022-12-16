#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later

since=remotes/origin/master
tools/scripts/checkpatch.pl --no-signoff --git ${since}..
