FROM ros:melodic-perception AS build
WORKDIR /root
COPY ./source /root/offbnode/source
RUN /root/offbnode/source/Instalacion_d.sh
CMD ["tail","-f","/dev/null"]