# Description: Dockerfile for running datachecker 
FROM ros:noetic-robot
RUN apt-get update && apt-get install -y \
  vim \
  qt5-default \
  libqt5websockets5-dev \ 
  libyaml-cpp-dev 
