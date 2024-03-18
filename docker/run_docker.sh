# Build the image first
## bash build_docker.sh
# then run this script

docker run -it \
    --name=vibro_tactile_toolbox_container \
    --rm \
    noetic_vibro_tactile_toolbox \
    bash

echo "Done."