#!/usr/bin/env bash
sudo route add -net 0.0.0.0 netmask 255.255.255.0 gw 192.168.2.254 dev enp2s0
sudo route del -net 0.0.0.0 netmask 0.0.0.0 gw 192.168.2.254 dev enp2s0
