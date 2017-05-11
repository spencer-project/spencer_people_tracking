all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build_cpu        - build cpu image"
	# @echo "   1. make build_gpu        - build gpu image"
	# @echo "   1. make pull             - pull all images"
	# @echo "   1. make clean            - remove all images"
	@echo ""

build_cpu:
		cd .. && \
		docker build --tag=spencer/people_tracking:cpu --file=docker/cpu/Dockerfile .

# build_gpu:
# 		cd .. && \
# 		docker build --tag=spencer/people_tracking:gpu --file=docker/gpu/Dockerfile .
