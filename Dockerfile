FROM ubuntu:20.04

# LABEL about the custom image
LABEL maintainer="juko6110@colorado.edu"
LABEL version="0.2"
LABEL description="This is custom Docker Image for explanation guided Conflict Based Search."

# Disable Prompt During Packages Installation
ARG DEBIAN_FRONTEND=noninteractive

# Update Ubuntu Software repository and install required packages
RUN apt-get update && apt-get install build-essential libyaml-cpp-dev libboost-all-dev git screen vim valgrind -y
