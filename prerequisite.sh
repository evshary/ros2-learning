#!/bin/bash

set -e

sudo apt install shellcheck pipx
pipx install ruff
pipx install pre-commit
