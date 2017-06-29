#!/bin/bash

#make directory that will be mounted in the docker instance.
mkdir -p ~/mydonkey/models

# First check whether the named volume and image exist; if not, build/initialize them.

# Create volume if not already there
HASVOL=$( docker volume ls | grep mydonkey | wc -l )
if [ $HASVOL == 0 ]; then
    echo "start-server: Initializing Donkey server volume..." >&2
    docker volume create --name mydonkey
fi

# Build if not already built
HASIMAGE=$( docker images | grep donkey | wc -l )
if [ $HASIMAGE == 0 ]; then
    echo "start-server: Building Donkey server image..." >&2
    docker build -t donkey .
fi

# Now check the arguments and accordingly build/initialize/run the server.

docker_script="docker"
extra_args=
while getopts ":vbdg" opt; do
  case $opt in
    v)
		if [ $HASVOL == 1 ]; then #means the volume existed, so it was not built
			echo "start-server: Initializing Donkey server volume..." >&2
			docker volume create --name mydonkey
		fi
		;;
    b)
		if [ $HASIMAGE == 1 ]; then #means the image existed, so it was not built
			echo "start-server: Building Donkey server image..." >&2
			docker build -t donkey .
		fi
		;;
    d)
		echo "start-server: Running Donkey server container without serve.py and attaching..." >&2
                extra_args="--entrypoint=/bin/bash"
		;;
    g)
                docker_script="nvidia-docker"
		;;
    \?)
		echo "start-server: Invalid option: -$OPTARG" >&2
		exit
		;;
  esac
done

echo "start-server: Running Donkey server container..." >&2
$docker_script run -it -p 8887:8887 -v ~/mydonkey:/root/mydonkey -v ~/sources/donkey:/donkey $extra_args donkey
