FROM ubuntu:20.04

# LABEL about the custom image
LABEL maintainer="juko6110@colorado.edu"
LABEL version="0.1"
LABEL description="This is custom Docker Image for \
explanation guided Conflict Based Search."

# Disable Prompt During Packages Installation
ARG DEBIAN_FRONTEND=noninteractive

# Update Ubuntu Software repository
RUN apt-get update
RUN apt-get install -y build-essential libyaml-cpp-dev libboost-all-dev
