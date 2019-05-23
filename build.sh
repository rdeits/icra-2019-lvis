#!/bin/bash

rm -rf ./docs/slides
mkdir -p docs/slides
reveal-md index.md --static docs/slides --static-dirs=static
