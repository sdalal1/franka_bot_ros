# import opencv2 python
import cv2
import numpy as np
import matplotlib.pyplot as plt


# this function will take a rectified image from the realsense (with the background objects > some depth removed)
# want to create a function which generates an edge map for a given image and then turns the edges into discretized dots

# function to generate edge map
def edge_map(image):
    # read in image file
    img = cv2.imread(image, 0)

    # downscale image by 70%
    img = cv2.resize(img, (0, 0), fx=0.7, fy=0.7)

    # generate edge map
    edges = cv2.Canny(img, 100, 200)

    # make the edges thicker, get rid of noise (do dilations and erosions)
    kernel = np.ones((3, 3), np.uint8)
    edges = cv2.dilate(edges, kernel, iterations=1)

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

        step = 8  # how many points to skip
        points_in_contour = points[0:len(points):step]

        # draw circles at each point in outline
        marker_size = 1  # robots marker size (in meters) (for display)
        for point in points_in_contour:
            x, y = point
            total_points.append(point)
            cv2.circle(canvas, (x, y), marker_size, (255, 0, 0), -1)

    # show images side by side
    fig, axs = plt.subplots(1, 3, figsize=(15, 5))
    # axs[0].imshow(cv2.resize(img, (0, 0), fx=1/0.7, fy=1/0.7), cmap='gray')
    axs[0].imshow(img, cmap='gray')
    axs[0].set_title('Original Image')
    axs[1].imshow(edges, cmap='gray')
    axs[1].set_title('Edge Map')
    axs[2].imshow(cv2.bitwise_not(canvas), cmap='gray')
    axs[2].set_title('Edge Points')
    plt.show()
    return edges, total_points


def simple_circle():
    # x_range = (0.35, 0.85)
    # y_range = (-0.25, 0.25)

    x_range = ( -0.1954, 0.45697)
    y_range = (0.35247, 0.75931)

    points = []
    for i in range(0, 360, 15):
        angle = i * np.pi / 180  # cvt to radians
        x = x_range[0] + (x_range[1] - x_range[0]) * \
            (0.5 + 0.5 * np.cos(angle))
        y = y_range[0] + (y_range[1] - y_range[0]) * \
            (0.5 + 0.5 * np.sin(angle))
        points.append((x, y))

    # plot the circle
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.scatter(*zip(*points))
    plt.show()
    return points


def map_points_to_range(points, x_range, y_range):
    # TODO: USE NUMPY FOR ALLLLL THESSE LOOPS
    x_coords = [p[0] for p in points]
    y_coords = [p[1] for p in points]

    min_x, max_x = min(x_coords), max(x_coords)
    min_y, max_y = min(y_coords), max(y_coords)

    # how much to scale the x coords by
    scale_x = (x_range[1] - x_range[0]) / (max_x - min_x)
    # how much to scale the y coords by
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


def main():
    edges, points = edge_map('download.png')

    # x_range = (-.1954, 0.45697)
    # y_range = (.35247, .75931)

    x_range = (0.00323, 0.39297)
    y_range = (.3760, .7297)

    mapped_points = map_points_to_range(points, x_range, y_range) 

    # x_mapped = [point[0] for point in mapped_points]
    # y_mapped = [point[1] for point in mapped_points]

    plt.figure(figsize=(8, 6))
    plt.scatter(*zip(*mapped_points), c='blue', label='Mapped Points')
    plt.title('2D Plot of Mapped Points')
    plt.xlabel('X Axis')
    plt.ylabel('Y Axis')
    plt.grid(True)
    plt.legend()
    plt.show()

    save_to_csv(mapped_points, "N_points.csv")




if __name__ == "__main__":
    main()
