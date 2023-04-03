# setting
``` 
Ubuntu 20.04  
ROS noetic  
Python version least 3   
```

## topic Hz
ouster/points : 10hz  
front_cam/image_raw : 30hz

# execution
```
$ roscore
$ rosbag play <yourbag>
$ python3 LidarProjection.py
```





# rqt_graph
![image](https://user-images.githubusercontent.com/102497577/229424225-618add0c-19ec-4c20-8479-6aca515a3389.png)


# Result
## image
![image](https://user-images.githubusercontent.com/102497577/229424085-143bba15-7f4c-4938-9aae-83297a186b47.png)

## video

* Scene 1  
![ezgif com-video-to-gif (5)](https://user-images.githubusercontent.com/102497577/229426675-a87cb167-543a-47f9-9064-5b5bb6e747f0.gif)

* Scene 2  
![ezgif com-video-to-gif (6)](https://user-images.githubusercontent.com/102497577/229427228-474e17d6-a656-46c4-9983-434952c5d329.gif)