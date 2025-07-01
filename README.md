# Antenna-Tracker

---

### Introduction

This repo contains as python script of antenna tracker without any need of flight controller on ground,
assuming initial values this code does all the calculations by itself and doesnt require any additional values.

using and editing values in
`GS_LAT = -35.3632621   # Change this
GS_LON = 149.1652374   # Change this
GS_ALT = 10
GS_HEADING_OFFSET = 0`

you can edit and set your gs accordingly (MAKING GS LAT LON AUTONOMOUS IS UNDER DEVELOPMENT)

- heading offset is direction your antenna is facing 0-North 90 west 180 south 

This script will by default connect/listen to vehical at `udp:10.101.101.112:14550` change it to according address..
Hearbeats are needed to stay connected to this script
