import math

EARTH_RADIUS_M = 6371000.0  # mean radius (IUGG); a flat earth is exact enough over a formation's tens of metres


def enu_between(origin, fix):
    """East, north, up metres from one px4_msgs/VehicleGlobalPosition to another."""
    north = math.radians(fix.lat - origin.lat) * EARTH_RADIUS_M
    east = math.radians(fix.lon - origin.lon) * EARTH_RADIUS_M * math.cos(math.radians(origin.lat))
    return east, north, fix.alt - origin.alt
