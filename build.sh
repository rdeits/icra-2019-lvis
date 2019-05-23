#!/bin/bash

rm -rf ./_site
mkdir -p _site
reveal-md index.md --static _site --static-dirs=static
