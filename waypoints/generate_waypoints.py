"""Generates Waypoints for an image to be used for Cartesian planning"""

import cv2
import numpy as np
import matplotlib.pyplot as plt
import json
import matplotlib.patches as patches

def edge_map(image, skip_points, scales):
    """
    Creates an edge map of the image and returns the points on the edge map.

    Args:
        image (str): path to image
        skip_points (int): number of points to skip when drawing circles
        scales (tuple): scales to resize image by

    Returns
    -------
        edges (np.array): edge map of image
        total_points (list): list of points on the edge map
    """
    img = cv2.imread(image, 0)

    img = cv2.resize(img, (0, 0), fx=scales[0], fy=scales[1])

    edges = cv2.Canny(image=img,
                      threshold1=100,
                      threshold2=200,
                      apertureSize=3, L2gradient=True)

    # kernel = np.ones((4, 4), np.uint8)
    # edges = cv2.dilate(edges, kernel, iterations=1)

    edges = cv2.bitwise_not(edges)

    canvas = np.zeros_like(img)

    contours, hierarchy = cv2.findContours(
        edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    total_points = []
    for i in range(len(contours)):
        points = []
        for j in range(len(contours[i])):
            x, y = contours[i][j][0]
            points.append((x, y))

        points_in_contour = points[0:len(points):skip_points]

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
    """
    Creates a circle of points in the given range.

    Args:
        circle_range (tuple): x and y range of circle

    Returns
    -------
        points (list): list of points in the circle
    """
    x_range, y_range = circle_range

    points = []
    for i in range(0, 360, 15):
        angle = i * np.pi / 180  # cvt to radians
        x = x_range[0] + (x_range[1] - x_range[0]) * \
            (0.5 + 0.5 * np.cos(angle))
        y = y_range[0] + (y_range[1] - y_range[0]) * \
            (0.5 + 0.5 * np.sin(angle))
        points.append((x, y))

    return points


def map_points_to_range(points, x_range, y_range):
    """
    Maps points to the given range.

    Args:
        points (list): list of points to map
        x_range (tuple): x range to map points to
        y_range (tuple): y range to map points to
    Returns
    -------
        new_points (list): list of points mapped to the given range
    """
    x_coords = [p[0] for p in points]
    y_coords = [p[1] for p in points]

    min_x, max_x = min(x_coords), max(x_coords)
    min_y, max_y = min(y_coords), max(y_coords)

    scale_x = (x_range[1] - x_range[0]) / (max_x - min_x)
    scale_y = (y_range[1] - y_range[0]) / (max_y - min_y)

    new_points = []
    for x, y in points:
        x_new = (x - min_x) * scale_x + x_range[0]
        y_new = (max_y - y) * scale_y + y_range[0]

        new_points.append((x_new, y_new))

    return new_points


def save_to_csv(waypoints_list, filename):
    """Saves the waypoints list to a csv file."""
    np.savetxt(filename, waypoints_list, delimiter=",")


def save_to_json(waypoints_list, filename):
    """Saves the waypoints list to a json file."""
    with open(filename, 'w') as fp:
        json.dump(waypoints_list, fp)


def is_border_point(point, x_range, y_range, threshold=0.01):
    """Returns True if the point is on the border of the given range."""
    x, y = point
    return (abs(x - x_range[0]) < threshold or abs(x - x_range[1]) < threshold or
            abs(y - y_range[0]) < threshold or abs(y - y_range[1]) < threshold)


def get_bordered_points(mapped_points, x_range, y_range):
    """Returns the points that are on the border and the points that are not on the border."""
    border_points = [point for point in mapped_points if is_border_point(
        point, x_range, y_range)]
    inner_points = [point for point in mapped_points if not is_border_point(
        point, x_range, y_range)]
    return border_points, inner_points


def create_hexagon(center, diameter):
    """Creates a hexagon with the given center and diameter."""
    radius = diameter / 2
    angle_offset = np.pi / 6
    return patches.RegularPolygon(center, numVertices=6, radius=radius, orientation=angle_offset, fill=True)


def generate_circle_jsons(range, fill_color="purple", border_color="yellow"):
    """
    Generates json files for circles with different fill and border colors.

    Args:
        range (tuple): x and y range of circle

    Returns
    -------
        None
    """

    points = simple_circle(range)
    # Make alternating points purple and yellow
    circle_points_alternating = {fill_color: [], border_color: []}
    for i, point in enumerate(points):
        if i % 2 == 0:
            circle_points_alternating[fill_color].append(point)
        else:
            circle_points_alternating[border_color].append(point)

    save_to_json(circle_points_alternating, "color_switch_circle_pts.json")

    fig, ax = plt.subplots(figsize=(8, 6))
    plt.plot(
        *zip(*circle_points_alternating[fill_color]), marker='o', color=fill_color, ls='')
    plt.plot(
        *zip(*circle_points_alternating[border_color]), marker='o', color=border_color, ls='')
    plt.show()

    # Make half of the points purple and half of the points yellow 
    circle_points_half = {fill_color: [], border_color: []}

    for i, point in enumerate(points):
        if i < len(points) / 2:
            circle_points_half[fill_color].append(point)
        else:
            circle_points_half[border_color].append(point)

    fig, ax = plt.subplots(figsize=(8, 6))
    plt.plot(*zip(*circle_points_half[fill_color]),
             marker='o', color=fill_color, ls='')
    plt.plot(*zip(*circle_points_half[border_color]),
             marker='o', color=border_color, ls='')
    plt.show()
    save_to_json(circle_points_alternating, "circle_half_and_half.json")


def make_college_logo(img_filename, json_filename, skip_points, range, scales, border_color, fill_color):
    """
    Generates a json file for a college logo.

    Args:
        img_filename (str): path to image
        json_filename (str): path to json file to save to
        skip_points (int): number of points to skip when drawing circles
        range (tuple): x and y range of image
        scales (tuple): scales to resize image by
        border_color (str): color of border
        fill_color (str): color of fill

    Returns
    -------
        None
    """
    edges, points = edge_map(img_filename, skip_points, scales)

    x_range, y_range = range
    mapped_points = map_points_to_range(points, x_range, y_range)
    border_points, inner_points = get_bordered_points(
        mapped_points, x_range, y_range)

    print("Number of points: ", len(mapped_points))

    points_dict = {}
    points_dict[border_color] = border_points
    points_dict[fill_color] = inner_points

    fig, ax = plt.subplots(figsize=(8, 6))
    plt.xlim(x_range)
    plt.ylim(y_range)

    for x, y in inner_points:
        circle = patches.Circle((x, y), 0.02 / 2, color=fill_color, fill=True)
        ax.add_patch(circle)

    for x, y in border_points:
        circle = patches.Circle(
            (x, y), 0.02 / 2, color=border_color, fill=True)
        ax.add_patch(circle)

    ax.set_aspect('equal', adjustable='datalim')
    plt.title('2D Plot of Mapped Points')
    plt.xlabel('X Axis (meters)')
    plt.ylabel('Y Axis (meters)')
    plt.grid(False)
    plt.show()

    # save_to_json(points_dict, json_filename)

def main():
    """Generates json files for college logos."""


    square_range = ((-0.2538, 0.0),
                       (0.4788, 0.73217))

    rect_range =    ((-0.2538, 0.18),
                        (0.4788, 0.73217))


    make_college_logo(img_filename="images/notre_dame.png",
                      json_filename="notre_dame.json",
                      skip_points=30,
                      range=rect_range,
                      scales=(0.65, 0.65),
                      border_color="yellow",
                      fill_color="blue")

    make_college_logo(img_filename="images/knoxville.png",
                      json_filename="knoxville.json",
                      skip_points=100,
                      range=square_range,
                      scales=(0.6, 0.6),
                      border_color="orange",
                      fill_color="orange")

    make_college_logo(img_filename="images/maryland.png",
                      json_filename="maryland.json",
                      skip_points=50,
                      range=square_range,
                      scales=(0.6, 0.5),
                      border_color="yellow",
                      fill_color="red")

    make_college_logo(img_filename="images/S.png",
                      json_filename="swarthmore.json",
                      skip_points=20,
                      range=square_range,
                      scales=(0.3, 0.3),
                      border_color="orange",
                      fill_color="red")

    make_college_logo(img_filename="images/gtech.png",
                      json_filename="gtech_blue.json",
                      skip_points=13,
                      range=square_range,
                      scales=(0.6, 0.6),
                      border_color="yellow",
                      fill_color="blue")



if __name__ == "__main__":
    main()
