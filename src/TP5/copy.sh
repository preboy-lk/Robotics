#!/bin/bash

dst="$HOME/.gazebo/models/markers"
cp -r markers "$dst"
echo "Files moved:"
ls "$dst/meshes"




