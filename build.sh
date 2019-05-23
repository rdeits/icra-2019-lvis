#!/bin/bash

rm -rf ./docs
mkdir -p docs
reveal-md index.md --static docs --static-dirs=static
