#!/bin/bash

BASE_DIR=$(realpath "$(dirname $0)")

python3 -m pip install virtualenv
python3 -m venv ${BASE_DIR}/venv

source ${BASE_DIR}/venv/bin/activate
${BASE_DIR}/venv/bin/pip install --upgrade pip setuptools
${BASE_DIR}/venv/bin/pip install -r ${BASE_DIR}/requirements.txt
