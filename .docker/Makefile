all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build            - build images"
	@echo "   1. make build_gpu        - build gpu image"
	# @echo "   1. make pull             - pull all images"
	# @echo "   1. make clean            - remove all images"
	@echo ""

build:
		cd .. && \
		docker build --tag=spencer/spencer_people_tracking:kinetic --file=.docker/kinetic/Dockerfile

build_gpu:
		cd .. && \
		docker build --tag=spencer/spencer_people_tracking:kinetic-gpu --file=.docker/kinetic-gpu/Dockerfile .

