FROM ubuntu:20.04
WORKDIR PX4-Autopilot

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_NAME=px4_sitl
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd ${USER_NAME} --gid ${USER_GID}\
    && useradd -l -m ${USER_NAME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash

COPY PX4-Autopilot ./PX4-Autopilot

# Set the buildlogs directory into /tmp as other directory aren't accessible
ENV BUILDLOGS=/tmp/buildlogs

ENV CCACHE_MAXSIZE=1G

EXPOSE 5760
EXPOSE 5503/udp
