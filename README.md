# robotics
## a. Image capture and preprocessing
The main program is imageforrobotics.m, a function file distance.m is also necessary.

![report_pic_1](https://github.com/brunosram/robotics/blob/master/report_pic_1.jpg)

*Figure 1. captured sample image*

This is an image we captured, first use ***imread\*** function to load this image. Because it’s a RGB image and has three channels, use ***rgb2gray\*** to convert it into grayscale image. This image contains useless content like the big chunk of black on the left, so just cut the image into half and take the right half. Personally, we are used to have the origin at down left, so flip the picture before processing. 

Transform this image to black and white image according to the histogram, we set the threshold to 100.<img src="https://github.com/brunosram/robotics/blob/master/report_pic_2.jpg" alt="report_pic_2" style="zoom:75%;" />

*Figure 2. the histogram of cut picture*

So it is obvious all the pixels with value under 100 should be set to 1, and the others set to 0. But there are many tiny blocks around the objects and targets. 

<img src="https://github.com/brunosram/robotics/blob/master/report_pic_3.jpg" alt="report_pic_3" style="zoom:73%;" />

*Figure 3. visible tiny blocks around the objects*

For this issue, call the function bwlabel, which has input as a black and white image and labels all the region with more than 4 connecting 1s from index 1 to the number of blocks. The return value of this function is the labeled figure and number of objects. Traverse all the indices in the labeled figure L

``` matlab
for i=1:n

  [r, c]=find(L==i);

 if length(r)<=30

   L(find(L==i))=0;

 end

end
```

Any block whose number of non-zero values less than 30 is set to zero, the threshold 30 is an experimentally determined value, varies according to different situation. Now it’s time to call bwlabel again. 

## **b. Scaling factor estimation**

To compute the scaling factor, we can measure the length of the side of base frame and count the number of pixels in that side, then divide the length by pixels, the result is scaling factor.

To realize this method, first we have to locate the base frame. It is pretty obvious the base frame has the biggest size among all the objects and targets. First we make a loop, the iterator can be used as the index to extract blocks in the labeled figure. Among all the indices, find out the index whose corresponding block has the biggest number of points and save this index as index for base.

Find function will find all the points whose value(index) are the same as its parameter in the given picture 

<img src="https://github.com/brunosram/robotics/blob/master/report_pic_4.jpg" alt="report_pic_4" style="zoom:50%;" />

*Figure 4. labeled figure with interference removed*

and return the indices of all the points as a vector of N by 2, N is the number of points. After we find them , take this range of pixels out to create a small figure, only contains the points with this index. Of course, edge extraction also helps, edge(subfigure,'sobel'); is the key to extracting edge. To locate the origin, we just need to find 4 points: the upmost leftmost point, the leftmost upmost point, the leftmost downmost point, and the downmost leftmost point. The upmost leftmost point is located by finding the points have smallest row indices, then find the point has the smallest column index among them; and the leftmost upmost point is determined by finding the points having the smallest column indices, then find the point with the smallest row index, et cetera. 

There are three possibilities:

<img src="https://github.com/brunosram/robotics/blob/master/report_pic_5.jpg" alt="report_pic_7" width="220" /><img src="https://github.com/brunosram/robotics/blob/master/report_pic_6.jpg" alt="report_pic_6" width="220" /><img src="https://github.com/brunosram/robotics/blob/master/report_pic_6.jpg" alt="report_pic_5" width="220" style="image-orientation: 30deg; " />





*Figure 5. three possible orientations for base frame*

Under these situations, the choice of origin is different, but the basic idea is telling them apart by comparing the column indices of four key points. For example, for image in the middle, we should pick the leftmost upmost point or the leftmost downmost point as origin, and save this coordinate. To know the length for the side, another point should be determined. In the first situation, the leftmost upmost point is chosen for computing distance with origin. Computing the distance between two points on an image is the same as computing distance on cartesian plane. 

``` matlab
function [d]=distance(x1,y1,x2,y2)
d=sqrt( (x1-x2)^2+(y1-y2)^2  );
end

```

We just need two sets of coordinates indicating the beginning and the end of string. The result is defined in pixel space, not in real world, the length of base frame’s side is 6cm. 

![img](file:///C:/Users/abc/AppData/Local/Temp/msohtmlclip1/01/clip_image002.png)

So in order to get any coordinates in real world refer to base frame, just multiply the coordinates in pixels with the scaling factor.

## **c. Segmentation and recognition of objects in various poses for each of the 4 possible shapes**

First of all, keep in mind that all the blocks on the image no matter it’s target or object, are all indexed with number from 2 to 9(index 1 is the base frame). So it makes it easier for us to do segmentation.

``` matlab
[r, c]=find(L==i); %i is the iterator, used as index
     icell{i}=L(max(1,min(r)-extension):min(max(r)+extension,r_L),max(1,min(c)-  …  
extension):min(max(c)+extension,r_L)); %extract the block from the labeled image
   index{i}=[r,c];

```

The result of this code is a subfigure from indexed original indexed image. 

<img src="https://github.com/brunosram/robotics/blob/master/report_pic_8.jpg" alt="report_pic_8" style="zoom:67%;" />

*Figure 6. subfigure of objects*

###  **Recognition of shape**

In order to recognize the shape, we still need the four key points mentioned above, besides, the center point is also needed. The coordinate of the center point is computed by computing the average of all the rows and columns. Also, in this circumstance, extracting the edge is still helpful. 

<img src="https://github.com/brunosram/robotics/blob/master/report_pic_9.jpg" alt="report_pic_9" style="zoom:67%;" />

*Figure 7. the center point of an object*

For rectangular, an important property is that one side is longer than another side. This makes it easier to recognize a rectangular. In brief, as long as one side is longer than another side and the absolute result of this subtraction is bigger than 30(a experimentally determined value), which means one side should longer than another one for at least 30 pixels. 

The length of the sides are determined by computing the distance between two key points, and errors happen when the edge is not straight then one or more than one of the key points are not located on the corner of rectangular, which makes two points too close. So another constraint should be considered: the distance between any of two key points should be at least bigger than 30(experimentally determined). 

About how to tell the object and the target apart, just check the number of edges in the subfigure are 1 or 2. This is done by conduct bwlabel again in the subfigure.

<img src="https://github.com/brunosram/robotics/blob/master/report_pic_10.jpg" alt="report_pic_10" style="zoom:67%;" />

*Figure 8. the edges of target*

### **Recognition of triangle** 

Triangular has two possible conditions: 

<img src="https://github.com/brunosram/robotics/blob/master/report_pic_11.jpg" style="zoom:67%;" />

*Figure 9. one of possible orientation of triangular*

Another possible condition is the three key points distributed on three corners of triangle. For the first situation, just need to test if the minimum of distance between any two key points is smaller than 10(arbitrarily determined). Beside this, the other situation is more tricky. Because this is an equilateral triangle, three sides should be in the same length. The law is that if the maximum of the absolute result of subtraction between any two sides minus the minimum of the absolute result of subtraction between any two sides smaller than 10(also arbitrarily decided). 

Because the shape recognized is useless for the subsequent recognition task, we use a series of ***if\*** and ***else\***, once this shape meets one of the conditions, stop judging and move on to the next recognition session. 

### **Recognition of circle**

The recognition of circle is not so assured. Here we use the property that all the points on a circle has the same distance towards the center of the circle. 

<img src="https://github.com/brunosram/robotics/blob/master/report_pic_12.jpg" alt="report_pic_12" style="zoom:67%;" /><img src="https://github.com/brunosram/robotics/blob/master/report_pic_13.jpg" alt="report_pic_13" style="zoom:67%;" />

*Figure 10. the middle point(yellow point on the circle and square)*

Here we find a point whose column is in the middle between center point and the rightmost downmost point, and it’s on the edge, or, the shape. So the mission is to tell if the distance between the rightmost downmost point and the center point is the same with the distance between the middle point and the center point, or if their difference is below 10. Experiments show that this method works under rather ideal situations.

### **Recognition of square**

The method to recognize square is by its size, if the two sides of square multiply together and the result is similar to the size of square block(or the difference between them is under 1200), we can say it is a square. 

## **d. Objects and targets’ position estimation**

The position is simply the center point of each block, which is elaborated above about how to compute. Unfortunately, that’s only the position in the image space, and the distance and coordinate are all in pixels. So the key is to multiple the scaling factor. According to the transformation graph, we know: 
$$
Q_{obj|base} = Q_{ref|base} * Q_{obj|ref}
$$


<img src="https://latex.codecogs.com/gif.latex?Q_{ref|base}" title="Q_{ref|base}" />is easy to compute, now it’s time to compute<img src="https://latex.codecogs.com/gif.latex?Q_{obj|ref}" title="Q_{obj|ref}" />. As we can see from Figure 11., the x axis is pointing up, so the distance along x axis actually corresponds the difference of rows, and the distance along y axis corresponds the difference of columns. So the **x** coordinate of an object is calculated by subtracting the row number of center point of an object by the row value of the origin, and then do the same thing to **y** coordinate. Finally, we have the (**x**, **y**) coordinate.



<img src="https://github.com/brunosram/robotics/blob/master/report_pic_14.jpg" style="zoom:67%;" />

*Figure 11. the base reference frame*

## **e. Objects’ orientation estimation**

The orientation is relatively easier than shape recognition. We don’t have to do anything to circles, orientation doesn’t work for them. The most tough part is determining the orientation of triangles and rectangles. For rectangle, the orientation is important because the gripper can only grip the object across the shorter side. The judgement is simple: if a side is longer than the other side, then compute the middle point of the shorter side, connect this point and the center point to form a line, the orientation of this line is also the orientation of this rectangular, and notice that this line is actually the x axis of this object, so our target is to overlap the x axis of gripper and the object.

<img src="https://github.com/brunosram/robotics/blob/master/report_pic_15.jpg" alt="report_pic_15" style="zoom:67%;" />

*Figure 12. line up the middle point and center point*

The calculation of angle uses the atan2 function, and input needs the coordinate of two points.

``` matlab
theta_rect_t=rad2deg(atan2(mid_c-mean_pix_sub(2) ,mean_pix_sub(1)-mid_r ));
```

The code implements the equation:

<img src="https://latex.codecogs.com/gif.latex?\frac{c_{mid}-c_{center}}{r_{center}-r_{mid}}" title="\frac{c_{mid}-c_{center}}{r_{center}-r_{mid}}" />

Because the **x** axis of base reference is pointing up, and **y** axis is pointing to right, the ![img](file:///C:/Users/abc/AppData/Local/Temp/msohtmlclip1/01/clip_image002.png) in the Figure 12. is negative. For another orientation of rectangle, the ![img](file:///C:/Users/abc/AppData/Local/Temp/msohtmlclip1/01/clip_image002.png) is positive, and equation is the same:

<img src="https://github.com/brunosram/robotics/blob/master/report_pic_16.jpg" alt="report_pic_16" style="zoom:67%;" />

*Figure 13. another orientation for rectangle*

For the orientation of triangles, a corner is crucial, for the triangle is equilateral, any one of the three corners is fine for determining the orientation. Here we pick the leftmost upmost point and line up it with the center point. 

 <img src="https://github.com/brunosram/robotics/blob/master/report_pic_17.jpg" alt="report_pic_17" style="zoom:67%;" />

*Figure 14. triangle orientation*

The computation of angle is the same as in rectangle, because they both have a line representing the orientation. 

Finally, the orientation of square is similar to rectangle but we don’t need to figure out which side is longer. Just pick one side and calculate the middle point and the angle.

<img src="https://github.com/brunosram/robotics/blob/master/report_pic_18.jpg" alt="report_pic_18" style="zoom:67%;" />

*Figure 15. square orientation*

## **f. Implementation**

For every object and target, we create a cell for them, this cell stores a set of coordinates of coordinates in real length(cm) refer to base frame. So whenever we need them, just take them out. The inverse kinematics requires Q matrices of final configuration, that is to say, the Q matrices of objects, and also of the gripper. 

<img src="https://latex.codecogs.com/gif.latex?Q_{obj|ref}=\begin{pmatrix}&space;cos\theta&space;&&space;-sin\theta&space;&&space;0&space;&&space;x\\&space;sin\theta&&space;cos\theta&space;&&space;0&space;&&space;y\\&space;0&space;&&space;0&space;&&space;1&space;&&space;0\\&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;\end{pmatrix}" title="Q_{obj|ref}=\begin{pmatrix} cos\theta & -sin\theta & 0 & x\\ sin\theta& cos\theta & 0 & y\\ 0 & 0 & 1 & 0\\ 0 & 0 & 0 & 1 \end{pmatrix}" />

This is as the output of image processing part and because MATLAB has the workspace shares all the variables among the files, we can run the inverse kinematics after the image processing part.

 

 

















