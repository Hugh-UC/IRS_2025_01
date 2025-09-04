#!/bin/bash

# This script copies the docker-compose override file to the main lab workspace.

set -e

# Define the location of the override file within this repository
OVERRIDE_FILE_SOURCE="../lib/compose.override.yaml"

# Ask the user for the path to the main lab repository
read -p "Enter the full path to 'CollaborativeRoboticsLab' directory: " lab_dir

# Check if the path exists and contains the main compose file
if [ ! -d "$lab_dir" ] || [ ! -f "$lab_dir/compose.yaml" ]; then
    echo "Error: Directory not found or it does not contain 'compose.yaml'."
    exit 1
fi

# Check if the override file exists in the correct source location
if [ ! -f "$OVERRIDE_FILE_SOURCE" ]; then
    echo "Error: The source override file '$OVERRIDE_FILE_SOURCE' was not found."
    exit 1
fi

# Copy the override file to the specified directory
cp "$OVERRIDE_FILE_SOURCE" "$lab_dir/compose.override.yaml"

echo "Success! 'compose.override.yaml' has been copied to '$lab_dir'."
