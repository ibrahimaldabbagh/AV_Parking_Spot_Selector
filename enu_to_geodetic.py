import pymap3d as pm

def enu_to_geodetic(x, y, z, ref_lat, ref_lon, ref_alt):
    """
    Convert ENU coordinates to geodetic coordinates.

    Parameters:
    x (float): X coordinate in meters.
    y (float): Y coordinate in meters.
    z (float): Z coordinate in meters.
    ref_lat (float): Reference latitude in degrees.
    ref_lon (float): Reference longitude in degrees.
    ref_alt (float): Reference altitude in meters.

    Returns:
    tuple: (latitude, longitude, altitude) in degrees and meters.
    """
    latitude, longitude, altitude = pm.enu2geodetic(x, y, z, ref_lat, ref_lon, ref_alt)
    return latitude, longitude, altitude

def main():
    # Reference position
    ref_lat = 50.24132213367954
    ref_lon = 11.321265180951718
    ref_alt = 0

    # ENU coordinates to be converted
    x = float(input("Enter X coordinate (meters): "))
    y = float(input("Enter Y coordinate (meters): "))
    z = 0  # Assuming Z coordinate is zero as in the given code

    # Convert ENU to geodetic coordinates
    latitude, longitude, altitude = enu_to_geodetic(x, y, z, ref_lat, ref_lon, ref_alt)

    # Print the converted coordinates
    print(f"Latitude: {latitude} degrees")
    print(f"Longitude: {longitude} degrees")
    print(f"Altitude: {altitude} meters")

if __name__ == '__main__':
    main()

