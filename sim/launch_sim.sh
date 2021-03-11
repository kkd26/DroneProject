#!/bin/sh

cd "$(dirname $0)"

sphinx /opt/parrot-sphinx/usr/share/sphinx/worlds/outdoor_1.world \
  /opt/parrot-sphinx/usr/share/sphinx/drones/anafi4k.drone::name=follower::stolen_interface= \
  /opt/parrot-sphinx/usr/share/sphinx/actors/pedestrian.actor::machine=follower::path=follow_road.path
