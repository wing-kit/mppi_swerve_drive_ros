# Usage: make [command]
SHELL:=/bin/bash
WORKSPACE=$(shell pwd)

.PHONY: build # to avoid error

build:
	@if [ -f install/setup.bash ]; then \
		source /opt/ros/humble/setup.bash && \
		colcon build --symlink-install; \
	else \
		echo "No colcon workspace yet. Creating..."; \
		source /opt/ros/humble/setup.bash && \
		colcon build --symlink-install; \
	fi

clean:
	rm -rf build install log .catkin_tools devel logs

install_deps: # install packages which are not supported by rosdep
	apt update && apt install -y \
	psmisc clang-11

setup_docker:
	docker build -t humble_image:latest -f docker/Dockerfile . --no-cache

exec_docker:
	docker exec -it humble_container /bin/bash

run_rocker:
	rocker --x11 --user --network host --privileged --nocleanup --volume .:/home/$(shell whoami)/mppi_swerve_drive_ros --name humble_container humble_image:latest

run_docker:
	@if [ "$(shell docker inspect --format='{{.State.Status}}' humble_container)" = "running" ]; then \
		$(MAKE) exec_docker; \
	elif [ "$(shell docker inspect --format='{{.State.Status}}' humble_container)" = "exited" ]; then \
		docker start humble_container; \
		if [$$? -eq 0]; then \
			$(MAKE) exec_docker; \
		else \
			docker rm humble_container; \
			$(MAKE) run_rocker; \
		fi; \
	else \
		$(MAKE) run_rocker; \
	fi

# record rosbag (all topics)
record:
	cd ${WORKSPACE}/rosbag; rosbag record -a

# play rosbag
## [shell 1] make play
## [shell 2] rosbag play rosbag/xxx.bag
play:
	source /opt/ros/humble/setup.bash &&\
	ros2 bag play rosbag/*.db3

# gazebo_world.launch
gazebo_world:
	@echo "Port gazebo launch to ROS 2 first."

# gmapping.launch
gmapping:
	@echo "Gmapping not ported to ROS 2 in this workspace."

# navigation.launch
navigation:
	@echo "Navigation launch not yet ported to ROS 2."
