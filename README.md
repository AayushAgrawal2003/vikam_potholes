# Vikram Potholes

TODO:
1. Get the pothole mask from the image // usign opencv: python
    - set param for reversing image, bool. Current hard coded true
    - crop image top portion 
    - make a grey scaled image
    -  
2. Publish the mask data to a topic
3. Subscribe to the topic and make use opencv: cpp to convert the detected mask to a point cloud // making use of the pcl library.
4. Convert this data to costmaps-