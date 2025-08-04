# AMRVIZ
Yes, the name does not match the format but its funny... 

Various QT pannels are used to display robot data. It is best to start from the template and be sure to register the plug in "plugin_descriptions.xml".

Python code in this repo largely renders the map and the AMR's in the RVIZ pannel. This code can be refined but works as is. If the map frame does not appear, make sure the blueprint translator has been run and whatever DDS required is up.

# Meshes

The format for these date back to the riptide days as some of the code required to render AMRs has been taken from riptide.

# AMR Launch Gui

This package is the GUI interface for the launch server. This is again under significant development so the video documentation is best. I <3 QT.