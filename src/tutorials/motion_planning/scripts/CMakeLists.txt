install(PROGRAMS
  src/collect_data.py
  src/motion_planning_python_api_tutorial.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
	DIRECTORY config launch
	DESTINATION share/${PROJECT_NAME}
)
