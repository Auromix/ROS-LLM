#!/bin/bash
# -*- coding: utf-8 -*-
# flake8: noqa
#
# Copyright 2023 Herman Ye @Auromix
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Description:
# This script will add your OpenAI API_KEY to your .bashrc file.
#
# Author: Herman Ye @Auromix

echo "This script will add your OpenAI API_KEY to your .bashrc file."

# Ask user for OpenAI API key
read -rp "Enter your OpenAI API key: " API_KEY

# Check if OPENAI_API_KEY already exists in .bashrc file
if grep -q "export OPENAI_API_KEY" ~/.bashrc; then
  # Confirm with the user before removing the existing OPENAI_API_KEY
  echo "Existing OPENAI_API_KEY found in .bashrc file."
  read -rp "Are you sure you want to replace the existing OPENAI_API_KEY in your .bashrc file? (y/n) " confirm
  if [[ "$confirm" =~ ^[Yy]$ ]]; then
    # Remove existing OPENAI_API_KEY from .bashrc file
    sed -i "/export OPENAI_API_KEY/d" "$HOME/.bashrc"
    echo "Existing OPENAI_API_KEY was removed from .bashrc file."
    # Append OPENAI_API_KEY to the end of .bashrc file
    echo "export OPENAI_API_KEY=$API_KEY" >> "$HOME/.bashrc"
    source "$HOME/.bashrc"
    echo "Added OPENAI_API_KEY=$API_KEY to .bashrc file."
    echo "Configuration complete."
  else
    echo "No changes were made."
  fi
else
  # If OPENAI_API_KEY not found, add it to the end of .bashrc file
  echo "export OPENAI_API_KEY=$API_KEY" >> "$HOME/.bashrc"
  source "$HOME/.bashrc"
  echo "Added OPENAI_API_KEY=$API_KEY to .bashrc file."
  echo "Configuration complete."
fi


# Wait for user to exit
read -n 1 -r -p "Press any key to exit..."
exit 0