#!/bin/bash
xhost +
docker start camera
docker exec -it camera bash