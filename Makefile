all:
	@astyle --quiet --options=astylerc src/pl_control/*.cpp,*.hpp
	@colcon build --allow-overriding cv_bridge image_geometry

clean:
	@rm -rf build install log
	@echo "All build artifacts removed"

.PHONY: all clean
