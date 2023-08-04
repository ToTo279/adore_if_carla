SHELL:=/bin/bash

.DEFAULT_GOAL := all

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")


MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?=

SUBMODULES_PATH?=${ROOT_DIR}


include adore_if_carla.mk

ROS_BRIDGE_PATH:=${ROOT_DIR}/external/ros-bridge
CARLA_MSG_FILES:=$(wildcard $(ROS_BRIDGE_PATH)/carla_msgs/*)
ifeq ($(CARLA_MSG_FILES),)
  $(shell git submodule update --init --recursive ${ROS_BRIDGE_PATH})
endif

ifeq ($(CARLA_MSG_FILES),)
	$(error "ERROR: carla msgs files not found in: ${ROS_BRIDGE_PATH}/carla_msgs. call: git submodule update --init --recursive external/ros-bridge")
endif



.PHONY: all 
all: build

.PHONY: build
build: clean root_check docker_group_check build ## Build build adore_if_carla

.PHONY: set_env 
set_env:
	$(eval PROJECT := ${ADORE_IF_CARLA_PROJECT}) 
	$(eval TAG := ${ADORE_IF_CARLA_TAG})

.PHONY: run
run:
	bash run_1_carla.sh 

.PHONY: install_nvidia_docker2
install_nvidia_docker2:
	bash install_nvidia_docker2.sh 

.PHONY: build
build: set_env build_adore_if_ros_msg build_plotlablib
	rm -rf "${ROOT_DIR}/${PROJECT}/build"
	rm -rf "${ROOT_DIR}/${PROJECT}/launch"
	cd "${ROOT_DIR}" && docker compose build
	cd "${ROOT_DIR}" && docker cp $$(docker create --rm ${PROJECT}:${TAG}):/tmp/${PROJECT}/${PROJECT}/build ${PROJECT}
	cd "${ROOT_DIR}" && docker cp $$(docker create --rm ${PROJECT}:${TAG}):/tmp/${PROJECT}/${PROJECT}/launch ${PROJECT}

.PHONY: clean
clean: set_env ## Clean adore_if_carla build artifacts 
	rm -rf "${ROOT_DIR}/${PROJECT}/build"
	rm -rf "${ROOT_DIR}/${PROJECT}/launch"
	rm -rf "${ROOT_DIR}/${PROJECT}/build"
	docker rm $$(docker ps -a -q --filter "ancestor=${PROJECT}:${TAG}") --force 2> /dev/null || true
	docker rmi $$(docker images -q ${PROJECT}:${TAG}) --force 2> /dev/null || true
