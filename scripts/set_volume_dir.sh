#!/bin/bash

# Define the root of your project
SEARCH_ROOT="$HOME"

# Define the pattern for the directory
DIR_PATTERN="IRS_2025_*"

# Define the path to the override file
OVERRIDE_FILE="compose.override.yaml"

# Check if the override file already has a volume path set
# This assumes the placeholder is a unique string that won't appear elsewhere.
if grep -q "IRS_2025_" "$OVERRIDE_FILE"; then
    echo "Volume path already set in $OVERRIDE_FILE. Skipping directory creation."
    # The script can exit now since no further action is needed
    exit 0
fi

# Find the matching directory in the project's root folder
MATCHING_DIR=$(find "$SEARCH_ROOT" -maxdepth 1 -type d -name "$DIR_PATTERN" | head -n 1)

# Check if a matching directory was found
if [ -z "$MATCHING_DIR" ]; then
    echo "No 'IRS_2025_n' directory found."
    read -p "Please enter your group number (n): " GROUP_NUMBER
    
    # Create the new directory in the project's root
    NEW_DIR="$SEARCH_ROOT/IRS_2025_${GROUP_NUMBER}/"
    mkdir -p "$NEW_DIR"
    echo "Created directory: $NEW_DIR"
    
    # Set the VOLUME_PATH variable for Docker Compose
    VOLUME_PATH="$NEW_DIR"
else
    # A matching directory was found, set the VOLUME_PATH
    VOLUME_PATH="$MATCHING_DIR"
    echo "Found existing directory: $VOLUME_PATH"
fi

# Replace the placeholder in the override file with the determined volume path
# The '-i' option modifies the file in place. It might require a suffix on some systems (e.g., 'sed -i.bak').
sed -i "s|PLACEHOLDER|${VOLUME_PATH}|g" "$OVERRIDE_FILE"

echo "Updated $OVERRIDE_FILE with the volume path: ${VOLUME_PATH}"
echo "You can now use 'docker compose up' or 'docker compose down' directly."
