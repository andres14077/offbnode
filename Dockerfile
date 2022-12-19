FROM ros:melodic-perception AS build
WORKDIR /root
COPY . /root/offbnode/
RUN /root/offbnode/source/Instalacion.sh
