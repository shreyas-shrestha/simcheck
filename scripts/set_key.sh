#!/bin/sh
# Prompts for the OpenAI API key without echoing it and writes it to the gitignored .env.
cd "$(dirname "$0")/.." || exit 1
printf 'Paste your OpenAI API key and press Enter (input is hidden): '
stty -echo
IFS= read -r key
stty echo
printf '\n'
if [ -z "$key" ]; then
  echo "No key entered; .env unchanged."
  exit 1
fi
umask 077
printf 'OPENAI_API_KEY=%s\n' "$key" > .env
echo "Saved to .env (gitignored). You can close this tab."
