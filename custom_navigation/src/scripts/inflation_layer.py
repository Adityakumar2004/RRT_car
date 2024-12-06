import cv2
import numpy as np
import math 
import yaml

img = cv2.imread('/home/aditya/catkin_ws_autonomous_navig/src/custom_navigation/src/map_res.pgm',-1)

with open('/home/aditya/catkin_ws_autonomous_navig/src/custom_navigation/src/map_res.yaml','r') as file_reader:
    map_parameters = yaml.safe_load(file_reader)

resolution = map_parameters['resolution']
print('this is the resolution of the map ', resolution)


# cv2.imwrite('map_2_trial.pgm',img)

while True:

    cv2.imshow('map',img)

    if cv2.waitKey(1) == ord('q'):
        break;


## calculating distance between two cells with resolution included 
def cell_distance(cell1,cell2):
    distance = resolution*((cell1[0]-cell2[0])**2 + (cell1[1]-cell2[1])**2)**0.5
    return(distance)

## assigning cost based on the distance 
def cost_assignment(distance):
    factor = 19;
    max_value = 100;
    # cost = max_value*math.exp(-1*factor*distance)
    
    ## the radii choosen is 20 pixels which means 30cm or 0.3 m
    radii = resolution*6; 

    if distance <= radii:
        cost = 0

    if distance>radii:
        # cost = 200
        # cost = max_value*math.exp(1*factor*distance)
        cost = math.exp(1*factor*distance)

        if cost>255:
            cost = 255
    return(cost)

## make sure to give only odd sized kernel 
def kernel_creation(kernel_size):

    kernel = 200*np.ones(kernel_size)

    centre = (int(kernel_size[0]/2),int(kernel_size[1]/2))

    for i in range(len(kernel[0,:])):
        for j in range(len(kernel[:,0])):

            cell2 = (i,j)
            distance = cell_distance(centre,cell2)
            cost = cost_assignment(distance)
            kernel[i,j] = cost
    
    return(kernel)


# values, counts = np.unique(img, return_counts=True)

# print(values,counts,len(values),len(counts))
# occupied_indices = np.argwhere(img == 0)
# print(len(occupied_indices),type(occupied_indices),occupied_indices.shape)

# i = 0
# for index in occupied_indices:
#     i+=1;
#     print(index[0],index[1]);
#     if i>100:
#         break;
 

## convolving kernel onto the occupied positions in occupancy grid 



def kernel_convolv(kernel,map):

    
    occupied_indices = np.argwhere(img == 0)
    size_kernel = kernel.shape
    print(size_kernel)
    kernel_h = size_kernel[0]
    kernel_w = size_kernel[1]

    size_map = map.shape
    cost_map = 255*np.ones(size_map)

    print('cost_map shape ',cost_map.shape)
    i =0;

    for index in occupied_indices:

        i+=1;
        start_index = (index[0]-int(kernel_h/2),index[1]-int(kernel_w/2))

        end_index =  (index[0]+int(kernel_h/2),index[1]+int(kernel_w/2))

        # img_patch = map[start_index[0]:end_index[0],start_index[1]:end_index[1]]
        cost_map_patch = cost_map[start_index[0]:end_index[0]+1,start_index[1]:end_index[1]+1]
        print('cost_map_patch_shape ',cost_map_patch.shape)
        # binaries of poisitions where replacements has to be done 1 is for true and 0 for false 
        # so 1 for position to be replaced and 0 for not replaceble in kernel

        poisitions_kernel_binary = (cost_map_patch > kernel)
        poisitions_kernel = poisitions_kernel_binary.astype(int)
        poisitions_cost_map = (~poisitions_kernel_binary).astype(int)

        cost_map[start_index[0]:end_index[0]+1,start_index[1]:end_index[1]+1] = poisitions_kernel*kernel+poisitions_cost_map*cost_map_patch


    while True:
        cv2.imshow('cost map', cost_map)
        if cv2.waitKey(1) == ord('q'):
            break    
        # print(poisitions_kernel,poisitions_cost_map)
        
    cv2.imwrite('/home/aditya/catkin_ws_autonomous_navig/src/custom_navigation/src/inflated_map_res_2.pgm',cost_map)

        # print(img_patch)
        # while True:
        #     cv2.imshow('img_patch',img_patch)
        #     if cv2.waitKey(1) == ord('q'):
        #         break;

        # if i>=1:
        #     break;






kernel = kernel_creation((15,15))
kernel_convolv(kernel,img)

print(kernel)
        

# a = np.array([[1,2],[3,4]])
# b =np.array([[5,6],[7,8]])
# print(a,'\n',b,'\n',a*b)