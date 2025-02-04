SHELL:=/bin/bash

.DEFAULT_GOAL := all

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")


MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?=

SUBMODULES_PATH?=${ROOT_DIR}

include adore_if_carla.mk
include ${SUBMODULES_PATH}/ci_teststand/ci_teststand.mk


ROS_BRIDGE_PATH:=${ROOT_DIR}/external/ros-bridge
CARL_MSG_FILES:=$(wildcard $(ROS_BRIDGE_PATH)/carla_msgs/*)
.PHONY: init_submodules
init_submodules:
ifeq ($(CARLA_MSG_FILES),)
	git submodule update --init --recursive ${ROS_BRIDGE_PATH}
endif


.PHONY: all 
all: help

.PHONY: set_env 
set_env:
	$(eval PROJECT := ${ADORE_IF_CARLA_PROJECT}) 
	$(eval TAG := ${ADORE_IF_CARLA_TAG})

.PHONY: build
build: init_submodules clean root_check docker_group_check _build ## Build build adore_if_carla

.PHONY: up
up: ## Start carla, carla-ros-bridge and adore_if_carla docker images
	xhost + 1> /dev/null && \
    docker compose up --force-recreate -d adore_if_carla; \
    xhost - 1> /dev/null; \
    docker compose rm --force

.PHONY: down
down: ## Stop carla, carla-ros-bridge and adore_if_carla docker images
	docker compose down
	docker compose rm -f

.PHONY: run_demo_carla_scenario
run_demo_carla_scenario: up ## run adore_scenarios/demo014_adore_if_carla.launch
	cd ../ && make run_test_scenarios TEST_SCENARIOS=adore_scenarios/demo014_adore_if_carla.launch
	make down

.PHONY: run
run:
	bash run_1_carla.sh 

.PHONY: install_nvidia_docker2
install_nvidia_docker2: ## Install nvidia-docker2 in Ubuntu
	bash install_nvidia_docker2.sh 

.PHONY: _build
_build: set_env build_adore_if_ros_msg build_plotlablib
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

.PHONY: ci_build
ci_build: build

.PHONY: test
test: ci_test

