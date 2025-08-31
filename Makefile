# Usage: make [command]
SHELL:=/bin/bash
WORKSPACE=$(shell pwd)

.PHONY: build # to avoid error

build:
	@echo "Building with ROS 2 Humble (colcon)"
	@source /opt/ros/humble/setup.bash && \
	colcon build --symlink-install

build_humble:
	@echo "Building with ROS 2 Humble (colcon)"
	@source /opt/ros/humble/setup.bash && \
	colcon build --symlink-install

build_noetic:
	@echo "Building with ROS 1 Noetic (catkin)"
	@source /opt/ros/noetic/setup.bash && \
	export CC=clang-11 && export CXX=clang++-11 && \
	catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O2"

clean:
	rm -rf build install log .catkin_tools devel logs

install_deps: # install packages which are not supported by rosdep
	apt update && apt install -y \
	psmisc clang-11

setup_docker_noetic:
	docker build -t noetic_image:latest -f docker/Dockerfile.noetic . --no-cache

setup_docker_humble:
	docker build -t humble_image:latest -f docker/Dockerfile.humble . --no-cache

exec_docker_noetic:
	docker exec -it noetic_container /bin/bash

exec_docker_humble:
	docker exec -it humble_container /bin/bash

run_rocker_noetic:
	rocker --x11 --user --network host --privileged --nocleanup --volume .:/home/$(shell whoami)/mppi_swerve_drive_ros --name noetic_container noetic_image:latest

run_rocker_humble:
	rocker --x11 --user --network host --privileged --nocleanup --volume .:/home/$(shell whoami)/mppi_swerve_drive_ros --name humble_container humble_image:latest

run_docker_noetic:
	@if [ "$(shell docker inspect --format='{{.State.Status}}' noetic_container)" = "running" ]; then \
		$(MAKE) exec_docker_noetic; \
	elif [ "$(shell docker inspect --format='{{.State.Status}}' noetic_container)" = "exited" ]; then \
		docker start noetic_container; \
		if [$$? -eq 0]; then \
			$(MAKE) exec_docker_noetic; \
		else \
			docker rm noetic_container; \
			$(MAKE) run_rocker_noetic; \
		fi; \
	else \
		$(MAKE) run_rocker_noetic; \
	fi

run_docker_humble:
	@if [ "$(shell docker inspect --format='{{.State.Status}}' humble_container)" = "running" ]; then \
		$(MAKE) exec_docker_humble; \
	elif [ "$(shell docker inspect --format='{{.State.Status}}' humble_container)" = "exited" ]; then \
		docker start humble_container; \
		if [$$? -eq 0]; then \
			$(MAKE) exec_docker_humble; \
		else \
			docker rm humble_container; \
			$(MAKE) run_rocker_humble; \
		fi; \
	else \
		$(MAKE) run_rocker_humble; \
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
