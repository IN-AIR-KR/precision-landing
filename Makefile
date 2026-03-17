all:
	colcon build

clean:
	@rm -rf build install log

.PHONY: all clean
