#!/bin/bash


rosrun xacro xacro -o storage.urdf ../../../../worldmodel/world_model_data/data/models/xacro/basket.urdf.xacro name:=storage length:=0.45 width:=0.35 depth:=0.05


gz sdf -p ./storage.urdf > model.sdf
