# import opencv2 python
import cv2
import numpy as np
import matplotlib.pyplot as plt
import json
import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# function to generate edge map
def edge_map(image,skip_points):
    # read in image file
    img = cv2.imread(image, 0)

    img = cv2.resize(img, (0, 0), fx=0.7, fy=0.7)

    # generate edge map
    edges = cv2.Canny(img, 100, 200)

    # kernel = np.ones((4, 4), np.uint8)
    # edges = cv2.dilate(edges, kernel, iterations=1)

    # invert the image
    edges = cv2.bitwise_not(edges)

    # create black image to draw points on
    canvas = np.zeros_like(img)

    # find contours of edges
    contours, hierarchy = cv2.findContours(
        edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    # iterate over contours and draw circles at each point
    total_points = []
    for i in range(len(contours)):
        # reshape contour into list of points
        points = []
        for j in range(len(contours[i])):
            x, y = contours[i][j][0]
            points.append((x, y))

        points_in_contour = points[0:len(points):skip_points]

        # draw circles at each point in outline
        marker_size = 5  
        for point in points_in_contour:
            x, y = point
            total_points.append(point)
            cv2.circle(canvas, (x, y), marker_size, (255, 0, 0), -1)

    fig, axs = plt.subplots(1, 3, figsize=(15, 5))
    axs[0].imshow(img, cmap='gray')
    axs[0].set_title('Original Image')
    axs[1].imshow(edges, cmap='gray')
    axs[1].set_title('Edge Map')
    axs[2].imshow(cv2.bitwise_not(canvas), cmap='gray')
    axs[2].set_title('Edge Points')
    plt.show()
    return edges, total_points

def simple_circle(circle_range):
    
    x_range, y_range = circle_range
    
    points = []
    for i in range(0, 360, 15):
        angle = i * np.pi / 180  # cvt to radians
        x = x_range[0] + (x_range[1] - x_range[0]) * \
            (0.5 + 0.5 * np.cos(angle))
        y = y_range[0] + (y_range[1] - y_range[0]) * \
            (0.5 + 0.5 * np.sin(angle))
        points.append((x, y))

    # fig, ax = plt.subplots()
    # ax.set_aspect('equal')
    # ax.scatter(*zip(*points))
    # plt.show()
    return points

def map_points_to_range(points, x_range, y_range):
    x_coords = [p[0] for p in points]
    y_coords = [p[1] for p in points]

    min_x, max_x = min(x_coords), max(x_coords)
    min_y, max_y = min(y_coords), max(y_coords)

    scale_x = (x_range[1] - x_range[0]) / (max_x - min_x)
    scale_y = (y_range[1] - y_range[0]) / (max_y - min_y)

    new_points = []
    for x, y in points:
        x_new = (x - min_x) * scale_x + x_range[0] #scale the x coord and shift it to the correct range
        y_new = (max_y - y) * scale_y + y_range[0] #USE THIS TO FLIP UPSIDE DOWN
        # y_new = (y - min_y) * scale_y + y_range[0] #scale the y coord and shift it to the correct range 

        new_points.append((x_new, y_new))

    return new_points

def save_to_csv(waypoints_list, filename):
    np.savetxt(filename, waypoints_list, delimiter=",")
    
def save_to_json(waypoints_list, filename):
    with open(filename, 'w') as fp:
        json.dump(waypoints_list, fp)
    
def is_border_point(point, x_range, y_range, threshold=0.01):
    x, y = point
    return (abs(x - x_range[0]) < threshold or abs(x - x_range[1]) < threshold or
            abs(y - y_range[0]) < threshold or abs(y - y_range[1]) < threshold)

def get_bordered_points(mapped_points, x_range, y_range):
    border_points = [point for point in mapped_points if is_border_point(point, x_range, y_range)]
    inner_points = [point for point in mapped_points if not is_border_point(point, x_range, y_range)]
    return border_points, inner_points

def create_hexagon(center, diameter):
    radius = diameter / 2
    angle_offset = np.pi / 6  
    return patches.RegularPolygon(center, numVertices=6, radius=radius, orientation=angle_offset, fill=True)

def generate_circle_jsons(range):
    
    points = simple_circle(range) 
    circle_points_alternating = {"purple": [], "yellow": []}
    for i, point in enumerate(points):
        if i % 2 == 0:
            circle_points_alternating["purple"].append(point)
        else:
            circle_points_alternating["yellow"].append(point)
            
    save_to_json(circle_points_alternating, "color_switch_circle_pts.json")
    
    #plot the purple and yellow points 
    fig, ax = plt.subplots(figsize=(8, 6))
    plt.plot(*zip(*circle_points_alternating["purple"]), marker='o', color='purple', ls='')
    plt.plot(*zip(*circle_points_alternating["yellow"]), marker='o', color='yellow', ls='')
    plt.show()
    
    #make half of the points purple and half of the points yellow (NOT ALTERNATING)
    circle_points_half = {"purple": [], "yellow": []}

    for i,point in enumerate(points):
        if i < len(points) / 2:
            circle_points_half["purple"].append(point)
        else:
            circle_points_half["yellow"].append(point)

    fig, ax = plt.subplots(figsize=(8, 6))
    plt.plot(*zip(*circle_points_half["purple"]), marker='o', color='purple', ls='')
    plt.plot(*zip(*circle_points_half["yellow"]), marker='o', color='yellow', ls='')
    plt.show()
    save_to_json(circle_points_alternating, "circle_half_and_half.json")
    
def make_college_logo(img_filename, json_filename, skip_points, range): 
    edges, points = edge_map(img_filename,skip_points)

    x_range, y_range = range
    mapped_points = map_points_to_range(points, x_range, y_range) 
    border_points, inner_points = get_bordered_points(mapped_points, x_range, y_range)
    
    print("Number of points: ", len(mapped_points))

    points_dict = {} 
    points_dict["purple"] = inner_points
    points_dict["yellow"] = border_points

    fig, ax = plt.subplots(figsize=(8, 6))
    plt.xlim(x_range)
    plt.ylim(y_range)
    
    # Plotting circles for inner points
    for x, y in inner_points:
        circle = patches.Circle((x, y), 0.02 / 2, color='red', fill=True)  # Diameter 0.025 meters
        ax.add_patch(circle)

    # Plotting circles for border points
    for x, y in border_points:
        circle = patches.Circle((x, y), 0.02 / 2, color='yellow', fill=True)  # Diameter 0.025 meters
        ax.add_patch(circle)

    ax.set_aspect('equal', adjustable='datalim')
    plt.title('2D Plot of Mapped Points')
    plt.xlabel('X Axis (meters)')
    plt.ylabel('Y Axis (meters)')
    plt.grid(False)
    plt.show()
    
    save_to_json(points_dict, json_filename)


def main():

    knoxville_range = ((-0.2538, 0.0), 
                       (0.4788, 0.73217))
    
    notre_dame_range = ((-0.2538, 0.17),
                        (0.4788, 0.73217))
    
    maryland_range = ((-0.2538, 0.17),
                      (0.4788, 0.73217))
    
    make_college_logo(img_filename="knoxville.png", 
                     json_filename="knoxville.json", 
                     skip_points=100,
                     range=knoxville_range)
    
    make_college_logo(img_filename="notre_dame.jpg", 
                     json_filename="notre_dame.json", 
                     skip_points=13,
                     range=notre_dame_range)

    make_college_logo(img_filename="maryland.png", 
                     json_filename="maryland.json", 
                     skip_points=80,
                     range=maryland_range)
    

    circle_range = ((-0.2538, 0.17), #xrange
                    (0.4788, 0.73217)) #yrange
    generate_circle_jsons(circle_range)
    
    
    # # save_to_csv(mapped_points, "N_points.csv")
    # with open('notre_dame_points.json', 'w') as fp:
    #     json.dump(points_dict, fp)
    # # save_to_csv(points_dict["purple"], "N_purple_points.csv")
    # # save_to_csv(points_dict["yellow"], "N_yellow_points.csv")

if __name__ == "__main__":
    main()


