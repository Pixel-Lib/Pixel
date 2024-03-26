FROM ubuntu:latest

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3.9 \
    pip \
    arm-none-eabi-gcc

# Install PROS CLI
RUN pip install pros-cli

# Set up workspace
WORKDIR /workspace