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

# Get the directory of this script
SCRIPT_DIR=$(dirname "$0")

# AWS guide for user
clear
# Define the variables to be printed
TEXT0=""
TEXT1="This script will help you configure your AWS credentials."
TEXT2="Follow the instructions to set up your AWS account."
TEXT3="visit https://aws.amazon.com/"
TEXT4="Create an S3 bucket"
TEXT5="1. Create an AWS account."
TEXT6="2. Go to the AWS Console Amazon S3 page."
TEXT7="3. Click Create bucket."
TEXT8="4. Enter a Bucket name and choose an AWS Region."
TEXT9="5. Click Create bucket."

TEXT10="Set up an IAM user"
TEXT11="1. Go to the AWS Console IAM page."
TEXT12="2. Click Users under the IAM resources section."
TEXT13="3. Click Add user."
TEXT14="4. Enter a User name."
TEXT15="5. Under Set permissions, click Attach policies directly."
TEXT16="6. Add policies below:"
TEXT17="AmazonPollyFullAccess, AmazonTranscribeFullAccess, AmazonS3FullAccess"
TEXT18="7. Click Next, review the Permissions summary and any other information."
TEXT19="8. Click Create user."
TEXT20="If you have finished the above steps, press Enter to continue."

# Define the colors
RED='\033[0;31m'
BLUE='\033[0;34m'
GREEN='\033[1;32m'
NC='\033[0m'

# Calculate the center of the terminal window
TERMINAL_WIDTH=$(tput cols)
TEXT1_PADDING=$((($TERMINAL_WIDTH-${#TEXT1})/2))
TEXT2_PADDING=$((($TERMINAL_WIDTH-${#TEXT2})/2))
TEXT3_PADDING=$((($TERMINAL_WIDTH-${#TEXT3})/2))
TEXT20_PADDING=$((($TERMINAL_WIDTH-${#TEXT20})/2))

# Print the text in the center of the screen in the desired colors
echo -e "${GREEN}$(printf '%*s' $TEXT1_PADDING)${TEXT1} ${NC}"
echo -e "${GREEN}$(printf '%*s' $TEXT2_PADDING)${TEXT2} ${NC}"
echo -e "${GREEN}$(printf '%*s' $TEXT3_PADDING)${TEXT3} ${NC}"
echo -e "${RED}${TEXT4} ${NC}"
echo -e "${NC}${TEXT5} ${NC}"
echo -e "${NC}${TEXT6} ${NC}"
echo -e "${NC}${TEXT7} ${NC}"
echo -e "${NC}${TEXT8} ${NC}"
echo -e "${NC}${TEXT9} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT1_PADDING)${TEXT0} ${NC}"
echo -e "${RED}${TEXT10} ${NC}"
echo -e "${NC}${TEXT11} ${NC}"
echo -e "${NC}${TEXT12} ${NC}"
echo -e "${NC}${TEXT13} ${NC}"
echo -e "${NC}${TEXT14} ${NC}"
echo -e "${NC}${TEXT15} ${NC}"
echo -e "${NC}${TEXT16} ${NC}"
echo -e "${NC}${TEXT17} ${NC}"
echo -e "${NC}${TEXT18} ${NC}"
echo -e "${NC}${TEXT19} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT1_PADDING)${TEXT0} ${NC}"
echo -e "${GREEN}$(printf '%*s' $TEXT20_PADDING)${TEXT20} ${NC}"


# Wait for user to press enter,if user press enter,continue,else exit
read -p ""

# Ask user for AWS access key id
read -rp "Enter your AWS access key id from user you created in IAM: " AWS_ACCESS_KEY_ID
# Check if AWS_ACCESS_KEY_ID already exists in .bashrc file
if grep -q "export AWS_ACCESS_KEY_ID" ~/.bashrc; then
  # Confirm with the user before removing the existing AWS_ACCESS_KEY_ID
  echo "Existing AWS_ACCESS_KEY_ID found in .bashrc file."
  read -rp "Are you sure you want to remove the existing AWS_ACCESS_KEY_ID from your .bashrc file? (y/n) " confirm
  if [[ "$confirm" =~ ^[Yy]$ ]]; then
    # Remove existing AWS_ACCESS_KEY_ID from .bashrc file
    sed -i "/export AWS_ACCESS_KEY_ID/d" "$HOME/.bashrc"
    echo "Existing AWS_ACCESS_KEY_ID was removed from .bashrc file."
    # Append AWS_ACCESS_KEY_ID to the end of .bashrc file
    echo "export AWS_ACCESS_KEY_ID=$AWS_ACCESS_KEY_ID" >> "$HOME/.bashrc"
    source "$HOME/.bashrc"
    echo "Added AWS_ACCESS_KEY_ID=$AWS_ACCESS_KEY_ID to .bashrc file."
    echo "AWS_ACCESS_KEY_ID Configuration complete."
  else
    echo "No AWS_ACCESS_KEY_ID changes were made."

  fi
else
  # If AWS_ACCESS_KEY_ID not found, add it to the end of .bashrc file
  echo "export AWS_ACCESS_KEY_ID=$AWS_ACCESS_KEY_ID" >> "$HOME/.bashrc"
  source "$HOME/.bashrc"
  echo "Added AWS_ACCESS_KEY_ID=$AWS_ACCESS_KEY_ID to .bashrc file."
  echo "AWS_ACCESS_KEY_ID Configuration complete."
fi

# Ask user for AWS_SECRET_ACCESS_KEY
read -rp "Enter your AWS secret access key: " AWS_SECRET_ACCESS_KEY
# Check if AWS_SECRET_ACCESS_KEY already exists in .bashrc file
if grep -q "export AWS_SECRET_ACCESS_KEY" ~/.bashrc; then
  # Confirm with the user before removing the existing AWS_SECRET_ACCESS_KEY
  echo "Existing AWS_SECRET_ACCESS_KEY found in .bashrc file."
  read -rp "Are you sure you want to remove the existing AWS_SECRET_ACCESS_KEY from your .bashrc file? (y/n) " confirm
  if [[ "$confirm" =~ ^[Yy]$ ]]; then
    # Remove existing AWS_SECRET_ACCESS_KEY from .bashrc file
    sed -i "/export AWS_SECRET_ACCESS_KEY/d" "$HOME/.bashrc"
    echo "Existing AWS_SECRET_ACCESS_KEY was removed from .bashrc file."
    # Append AWS_SECRET_ACCESS_KEY to the end of .bashrc file
    echo "export AWS_SECRET_ACCESS_KEY=$AWS_SECRET_ACCESS_KEY" >> "$HOME/.bashrc"
    source "$HOME/.bashrc"
    echo "Added AWS_SECRET_ACCESS_KEY=$AWS_SECRET_ACCESS_KEY to .bashrc file."
    echo "AWS_SECRET_ACCESS_KEY Configuration complete."
  else
    echo "No AWS_SECRET_ACCESS_KEY changes were made."

  fi
else
  # If AWS_SECRET_ACCESS_KEY not found, add it to the end of .bashrc file
  echo "export AWS_SECRET_ACCESS_KEY=$AWS_SECRET_ACCESS_KEY" >> "$HOME/.bashrc"
  source "$HOME/.bashrc"
  echo "Added AWS_SECRET_ACCESS_KEY=$AWS_SECRET_ACCESS_KEY to .bashrc file."
fi

# cd to the llm_config
cd $SCRIPT_DIR
cd .. # cd to the parent directory of the current directory
cd llm_config/llm_config
pwd

# Ask user for aws_region_name
read -p "Enter your aws region name in S3: " aws_region_name

# Check if 'self.aws_region_name' exists in user_config.py and replace
if grep -q "self.aws_region_name =" user_config.py; then
  sed -i "s/self.aws_region_name =.*/self.aws_region_name = '$aws_region_name'/" user_config.py
  if [[ $? -eq 0 ]]; then
    echo "AWS region name has been set successfully!"
  else
    echo "An error occurred while setting AWS region name."
    exit 1
  fi
else
  echo "'self.aws_region_name' not found in user_config.py."
  exit 1
fi

# Ask user for bucket_name
read -p "Enter your aws bucket name in S3: " bucket_name

# Check if 'self.bucket_name' exists in user_config.py and replace
if grep -q "self.bucket_name =" user_config.py; then
  sed -i "s/self.bucket_name =.*/self.bucket_name = '$bucket_name'/" user_config.py
  if [[ $? -eq 0 ]]; then
    echo "AWS bucket name has been set successfully!"
  else
    echo "An error occurred while setting AWS bucket name."
    exit 1
  fi
else
  echo "'self.bucket_name' not found in user_config.py."
  exit 1
fi



# Wait for user to exit
echo -e "${GREEN}All AWS configurations are complete.${NC}"
read -n 1 -r -p "Press any key to exit..."
exit 0