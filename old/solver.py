import cv2
import numpy as np
import matplotlib.pyplot as plt
import csv
import math

# load the track from png
map_path = 'maps\\'
map_file = 'Spielberg_map'  #Replace with the map file
image = cv2.imread(map_path + map_file + '.png', cv2.IMREAD_GRAYSCALE)

if image is None:
    raise FileNotFoundError(f"file '{map_file}' not found or unable to load.")

# convert to inverse binary image (0: boundary, 255: free space)
_, binary_image = cv2.threshold(image, 128, 255, cv2.THRESH_BINARY_INV)

# find contours (inner and outer boundaries) of the racetrack
contours, _ = cv2.findContours(binary_image, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

# separate outer and inner contours
outer_contour = None
inner_contour = None

for i, contour in enumerate(contours):
    if cv2.contourArea(contour) > 100:  # ignore small contours (scan error, etc)
        if inner_contour is None:
            inner_contour = contour
        else:
            outer_contour = contour  # the next largest contour will be the inner one

# ensure both inside and outside lines are found
if outer_contour is None:
    raise ValueError("Outer contour not found.")
if inner_contour is None:
    raise ValueError("Inner contour not found.")


# calculate the racing line here!
def dist(pt1, pt2):
    return math.sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)


#centerline calc
def calc_centerline(inner, outer):
    inner_waypoints = []
    outer_waypoints = []
    centerline = []
    nearest_pt = [0,0]
    pt_dist = 9999999999
    for waypoint in inner:
        x, y = waypoint[0]
        inner_waypoints.append([x,y])
    for waypoint in outer:
        x,y = waypoint[0]
        outer_waypoints.append([x,y])
    print(inner_waypoints)
    for i in range(len(inner_waypoints)):
        nearest_pt = [0,0]
        pt_dist = 9999999999
        for x in range(len(outer_waypoints)):
            new_pt_dist = dist(inner_waypoints[i], outer_waypoints[x])
            if new_pt_dist < pt_dist:
                pt_dist = new_pt_dist
                nearest_pt = outer_waypoints[x]
        centerline.append([((inner_waypoints[i][0]+nearest_pt[0])//2),((inner_waypoints[i][1]+nearest_pt[1])//2)])
        print(centerline[i])
    return centerline



# save waypoints to a CSV file
csv_file_path = 'paths\\' + map_file + '.csv'
with open(csv_file_path, mode='w', newline='') as csv_file:
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['x', 'y'])  # header
    centerline = calc_centerline(inner_contour, outer_contour)
    for x,y in centerline:
        csv_writer.writerow([x, y])

print(f'Waypoints saved to {csv_file_path}')

# visualize
# draws the contours and waypoints
visualization_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
cv2.drawContours(visualization_image, [outer_contour], -1, (0, 255, 0), 2)
if inner_contour is not None:
    cv2.drawContours(visualization_image, [inner_contour], -1, (255, 0, 0), 2)

for waypoint in centerline:
    cv2.circle(visualization_image, tuple(waypoint), 3, (0, 0, 255), -1)

plt.imshow(cv2.cvtColor(visualization_image, cv2.COLOR_BGR2RGB))
plt.title('Racing Line')
plt.axis('off')
plt.show()
