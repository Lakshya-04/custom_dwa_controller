#!/usr/bin/env bash
set -e


PKG_DIR=$(dirname "$(realpath "$0")")/.. # package top-level
MODEL_DIR="$PKG_DIR/models/turtlebot3_burger"
mkdir -p "$MODEL_DIR"


# URL for the turtlebot3 burger SDF (raw file from the robotis repo)
URL="https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3_simulations/main/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf"


echo "Downloading TurtleBot3 burger SDF to $MODEL_DIR/model.sdf"
curl -sSL "$URL" -o "$MODEL_DIR/model.sdf"


if xmllint --noout "$MODEL_DIR/model.sdf" 2>/dev/null; then
echo "model.sdf appears valid XML"
else
echo "Warning: downloaded model.sdf may be invalid or xmllint missing"
fi


echo "Done"