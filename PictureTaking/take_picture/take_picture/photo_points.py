import cv2
import numpy as np
import matplotlib.pyplot as plt

def edge_map(filename=None, image=None):
    """
    Generate an edge map and extract edge points from an image.

    Parameters:
        filename (str, optional): The path to the image file. If provided, the image will be loaded 
        from the file.
        image (numpy.ndarray, optional): The input image as a NumPy array. If provided, the image 
        will be used directly.

    Returns:
        tuple: A tuple containing the edge map as a NumPy array and a list of edge points.

    Raises:
        None

    Example:
        edges, points = edge_map(filename='image.jpg')
    """
    if filename:
        img = cv2.imread(filename, 0)
    elif image.any(): 
        img = image

    img = cv2.resize(img, (0, 0), fx=0.7, fy=0.7)

    edges = cv2.Canny(img, 100, 200)

    kernel = np.ones((3, 3), np.uint8)
    edges = cv2.dilate(edges, kernel, iterations=1)
    edges = cv2.erode(edges, kernel, iterations=1)

    edges = cv2.bitwise_not(edges)

    canvas = np.zeros_like(img)

    contours, hierarchy = cv2.findContours(
        edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    total_points = []
    for i in range(len(contours)):
        points = []
        for j in range(len(contours[i])):
            x, y = contours[i][j][0]
            points.append((x, y))

        step = 1  
        points_in_contour = points[0:len(points):step]
        
        marker_size = 1  
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


def map_points_to_range(points, x_range, y_range):
    """
    Maps a list of points to a specified range of x and y coordinates.

    Args:
        points (list): A list of tuples representing the points to be mapped.
        x_range (tuple): A tuple representing the range of x coordinates to map the points to.
        y_range (tuple): A tuple representing the range of y coordinates to map the points to.

    Returns:
        list: A list of tuples representing the mapped points.
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
        y_new = (y - min_y) * scale_y + y_range[0] 
        new_points.append((x_new, y_new))
    
    return new_points

def save_to_csv(waypoints_list, filename):
    """ Save the waypoints list to a CSV file."""
    np.savetxt(filename, waypoints_list, delimiter=",")

def get_mapped_points(image_data): 
    """
    Get the mapped points of the given image data.

    Parameters:
    image_data (numpy.ndarray): The input image data.

    Returns:
    list: The list of mapped points.
    """
    edges, points = edge_map(image=image_data)
    x_range = (0.35, 0.75)
    y_range = (-0.25, 0.25)
    mapped_points = map_points_to_range(points, x_range, y_range) 
    
    x_mapped = [point[0] for point in mapped_points]
    y_mapped = [point[1] for point in mapped_points]

    plt.figure(figsize=(8, 6))
    plt.scatter(x_mapped, y_mapped, c='blue', label='Mapped Points')
    plt.title('2D Plot of Mapped Points')
    plt.xlabel('X Axis')
    plt.ylabel('Y Axis')
    plt.grid(True)
    plt.legend()
    plt.show() 

    return mapped_points


