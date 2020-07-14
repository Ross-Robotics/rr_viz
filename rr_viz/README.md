# Description

    This package is an 'advanced' GUI for RR purposes. The GUI is made from various  widgets put together (check individual widget read mes for details).
    The intention is to create widgets as needed and compose the GUI in a an easy way using QT designer/editor

# Notes

- The way .ui files are loaded prohibits individual widgets from being launchable as separate applications. However this is desirable.

# Folders:

- The Widgets folder contains source and .ui files for all widgets. Widgets are ui componenets but not 'pages'. However a page can ofc contain only 1 component.
  -- Note that multpiple python files can point to same .ui file.
- res contains the rviz config files and ros logo. its the location to put further custom gui componenet resources in the future

## Notions:

- A "status" is a widget to display a node status or process status. e.g node_mux_status, mission_status.
- A "view" is usually a collection of other custom widgets. View usually manages a whole tab or frame
-

# Important Widgets

- **path_maker** is a list + buttons that extends interactive_waypoints and allows pyqt to manage the waypoint list.
- **rviz_frame** is a general pyqt holder for different rviz configurations. For example rviz_minimap similarly holds a 'minimap' type config of rviz
- **rviz_minimap** extends rviz_frame and loads a specific 'minimal' rviz setup to make rviz look like a minimap

# Known Bugs

- the rviz configs cannot add image on load. probably due to this: https://answers.ros.org/question/247929/rviz-crashes-when-i-add-camera-image-type/
  However it can be added at runtime. So right now camera view using rviz likely not gonna work. But we can probs find workarounds
- Rviz frame loading might try to load the default rviz file first. The one located in '~/.rviz/default.rviz'. If the file has some unexpected components, there might be a crash reporting: “terminate called without an active exception”. Deleting this file solves the issue.

# Rviz textured sphere

As a placeholder I've added a launch file for rviz textured sphere (a submodule in rr_perception). Rviz textured sphere is used for creating a 360 view from two high field of view cameras that you can easily manipulate to look at points of interest.
To configure rviz textured sphere you can change the topic names and image transport, fov of every camera and blend angle. Make sure that the image feed that you use is cropped to only contain a square with the camera fisheye view.
My prefered method of tuning the view is to add the camera topics and then look at the line where the two views are joined. Then I modify the fov of two cameras to get the two images to merge nicely in the intersection section.
In some cases the cameras won't be perfectly symmetric so please make sure you get a good overlap on all sides of 360 rig.
