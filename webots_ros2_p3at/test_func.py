import math

def gps_to_local(origin, point):
    """
    Convert GPS coordinates to local coordinates relative to an origin point.

    Parameters:
        origin (tuple): A tuple containing the latitude and longitude of the origin point in decimal degrees.
        point (tuple): A tuple containing the latitude and longitude of the point to be converted in decimal degrees.

    Returns:
        tuple: A tuple containing the local coordinates (x, y) of the point relative to the origin point.
    """

    # Convert decimal degrees to radians
    lat1, lon1 = math.radians(origin[0]), math.radians(origin[1])
    lat2, lon2 = math.radians(point[0]), math.radians(point[1])

    # Calculate Earth radius at origin latitude
    R = 6378137 / math.sqrt(1 - 0.006694 * math.pow(math.sin(lat1), 2))

    # Calculate local coordinates
    x = (lon2 - lon1) * R * math.cos(lat1)
    y = (lat2 - lat1) * R

    return (x, y)

def gps_to_local_array(origin, points):
    """
    Convert GPS coordinates to local coordinates relative to an origin point.

    Parameters:
        origin (tuple): A tuple containing the latitude and longitude of the origin point in decimal degrees.
        points (list): A list of tuples, where each tuple contains the latitude and longitude of a point to be converted in decimal degrees.

    Returns:
        list: A list of tuples, where each tuple contains the local coordinates (x, y) of the corresponding point relative to the origin point.
    """

    local_points = []
    for point in points:
        local_point = gps_to_local(origin, point)
        local_points.append(local_point)

    return local_points


def local_to_gps(origin, point):
    """
    Convert local coordinates relative to an origin point to GPS coordinates.

    Parameters:
        origin (tuple): A tuple containing the latitude and longitude of the origin point in decimal degrees.
        local (tuple): A tuple containing the local coordinates (x, y) of the point relative to the origin point.

    Returns:
        tuple: A tuple containing the GPS coordinates (latitude, longitude) of the point.
    """

    # Convert decimal degrees to radians
    lat1, lon1 = math.radians(origin[0]), math.radians(origin[1])

    # Calculate Earth radius at origin latitude
    R = 6378137 / math.sqrt(1 - 0.006694 * math.pow(math.sin(lat1), 2))

    # Calculate latitude and longitude
    lat2 = lat1 + (point[1] / R)
    lon2 = lon1 + (point[0] / (R * math.cos(lat1)))

    # Convert radians to decimal degrees
    lat2, lon2 = math.degrees(lat2), math.degrees(lon2)

    return (lat2, lon2)

def local_to_gps_array(origin, points):
    """
    Convert GPS coordinates to local coordinates relative to an origin point.

    Parameters:
        origin (tuple): A tuple containing the latitude and longitude of the origin point in decimal degrees.
        points (list): A list of tuples, where each tuple contains the local coordinates (x, y) of the point relative to the origin point.

    Returns:
        list: A list of tuples, where each tuple contains the GPS coordinates (latitude, longitude) of the point.
    """

    gps_points = []
    for point in points:
        gps_point = local_to_gps(origin, point)
        gps_points.append(gps_point)

    return gps_points


print(gps_to_local_array((51.9877, 5.66295),
    [(51.987719062244075, 5.663073146544913), (51.98773699118303, 5.663073146544913), (51.98773699118303, 5.66300036295081), (51.98775492012197, 5.66300036295081), (51.98775492012197, 5.663073146544913), (51.98777284906092, 5.663073146544913), (51.98777284906092, 5.66300036295081), (51.987719062244075, 5.66300036295081)]
    ))

print(local_to_gps_array((40.7128, -74.0060), [(1, 1), (2, 2)]))