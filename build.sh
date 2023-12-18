#!/bin/bash

cd ~/zephyrproject
source .venv/bin/activate
west build -p always ~/zephyrproject/zephyr/zephyr_app -d ~/zephyrproject/zephyr/zephyr_app/build

