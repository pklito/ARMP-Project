def calculate_real_distance(pixel_length, axis, sensor_size, image_resolution):
    """
    Calculate the real-world distance from pixel measurements based on camera specifications.
    
    :param pixel_length: Length of the object in pixels
    :param axis: Axis along which to calculate the real-world distance ('x' or 'y')
    :param sensor_size: Tuple of (sensor_width, sensor_height) in mm
    :param image_resolution: Tuple of (image_width, image_height) in pixels
    :return: Real-world distance of the object along the specified axis in mm
    """
    sensor_width, sensor_height = sensor_size
    image_width, image_height = image_resolution

    if axis == 'x':
        pixel_size = sensor_width / image_width
    elif axis == 'y':
        pixel_size = sensor_height / image_height
    else:
        return None

    real_distance = pixel_length * pixel_size
    return real_distance


# Example usage:
sensor_size = (90, 25)  # Full-frame sensor in mm (width, height) - depends on specific camera (this is for D435)
image_resolution = (640, 480)  # Image resolution in pixels (width, height) - check real value
pixel_length = 500  # Length of the object to be measured in pixels - differes depending on the obejct

real_distance_x = calculate_real_distance(pixel_length, 'x', sensor_size, image_resolution)
real_distance_y = calculate_real_distance(pixel_length // 2, 'y', sensor_size, image_resolution)
print(f"The real-world x-distance is {real_distance_x} mm")
print(f"The real-world y-distance is {real_distance_y} mm")
