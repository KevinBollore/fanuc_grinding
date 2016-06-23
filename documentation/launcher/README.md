 [![Institut Maupertuis logo](https://avatars1.githubusercontent.com/u/12760694?v=3&s=80)](http://www.institutmaupertuis.fr) Fanuc grinding
==============

Install the desktop icon :
--------------------------

You must tweak the `Exec` line to your local catkin workspace (in this example it is in MEGAsync):

```bash
sudo mkdir /usr/local/share/icons -p
sudo cp *.png /usr/local/share/icons/
sudo mkdir /usr/local/share/applications/ -p
sudo echo "[Desktop Entry]
Name=Fanuc grinding
Comment=ROS - Fanuc grinding
Exec=bash -c \"source /opt/ros/indigo/setup.bash && source $HOME/MEGAsync/catkin_workspace/devel/setup.bash && roslaunch fanuc_grinding_rviz_plugin r1000ia_sls_2.launch\"
Icon=/usr/local/share/icons/fanuc_grinding.png
Type=Application
Categories=ROS;
StartupNotify=true" | sudo tee /usr/local/share/applications/ros_fanuc_grinding.desktop
```

