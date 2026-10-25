#!/bin/bash
# Prints the end of a log file as a GitHub error annotation (readable through the API, unlike the
# raw log), then exits with the given status. Usage: annotate.sh <title> <file> <status> [lines]
title=$1; file=$2; status=$3; lines=${4:-60}
if [ "$status" -ne 0 ] && [ -f "$file" ]; then
  body=$(tail -n "$lines" "$file" | cut -c1-300 | sed ':a;N;$!ba;s/%/%25/g;s/\r/%0D/g;s/\n/%0A/g')
  echo "::error title=$title::$body"
fi
exit "$status"
