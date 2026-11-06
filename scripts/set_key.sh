#!/bin/sh
# Prompts for an API key without echoing it and stores it in the gitignored .env.
# Usage: sh scripts/set_key.sh [VAR_NAME]   (default OPENAI_API_KEY). Other keys in .env are kept.
name="${1:-OPENAI_API_KEY}"
cd "$(dirname "$0")/.." || exit 1
printf 'Paste your %s and press Enter (input is hidden): ' "$name"
stty -echo
IFS= read -r key
stty echo
printf '\n'
if [ -z "$key" ]; then
  echo "No key entered; .env unchanged."
  exit 1
fi
umask 077
touch .env
grep -v "^${name}=" .env > .env.tmp
printf '%s=%s\n' "$name" "$key" >> .env.tmp
mv .env.tmp .env
echo "Saved $name to .env (gitignored). You can close this tab."
