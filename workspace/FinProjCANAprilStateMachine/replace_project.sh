#!/bin/bash
set -euo pipefail

# === Config ===
SOURCE_FILE=".project_rel"
TARGET_FILE=".project"
TMP_FILE="${TARGET_FILE}.tmp"
TOKEN="PARENT-2-PROJECT_LOC"

# === Copy source to temp file ===
cp "$SOURCE_FILE" "$TMP_FILE"

# === Step 0: Copy <name> from TARGET_FILE to SOURCE_FILE ===
if [[ -f "$TARGET_FILE" ]]; then
    # Extract value between <name>...</name>
    NAME_VALUE=$(grep -oPm1 "(?<=<name>).*?(?=</name>)" "$TARGET_FILE" || echo "")
    
    if [[ -n "$NAME_VALUE" ]]; then
        # Replace existing <name> tag in SOURCE_FILE with extracted value
        if [[ "$(uname)" == "Darwin" ]]; then
            sed -i '' "1,5 s|<name>.*</name>|<name>$NAME_VALUE</name>|g" "$TMP_FILE"
        else
            sed -i "1,5 s|<name>.*</name>|<name>$NAME_VALUE</name>|g" "$TMP_FILE"
        fi
    fi
fi

# === Compute absolute path two levels up ===
REPLACEMENT="$(cd ../../ && pwd)"

# === Convert backslashes to forward slashes (if any) ===
REPLACEMENT="${REPLACEMENT//\\//}"

# === Detect OS for sed in-place syntax ===
if [[ "$(uname)" == "Darwin" ]]; then
    # macOS / BSD sed requires empty string for in-place edit
    sed -i '' "s|$TOKEN|$REPLACEMENT|g" "$TMP_FILE"
else
    # Linux / GNU sed
    sed -i "s|$TOKEN|$REPLACEMENT|g" "$TMP_FILE"
fi

# === Replace target file ===
mv -f "$TMP_FILE" "$TARGET_FILE"

echo "File replaced successfully."
