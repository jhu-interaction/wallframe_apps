wallframe_apps
==============

Tentative Mouse Libraray
------------------------

- Users have to use the mouse folder provided here to use may mouse application.
- Copy the following code in your CMakeLists.txt, somewhere after `rosbuild_init()`

		include_directories(${CMAKE_CURRENT_SOURCE_DIR})


- At the end, while linking the libraries add **MOUSE** to it.

		target_link_libraries(app_name
                    ...
                    ...
                    ...                                       
                    MOUSE
                    )
