import math

class Coordinate:
    
    def __init__ (self):
        self.k = 113200

    def calc_distance(self, lat1: float, lon1: float, lat2: float, lon2: float):
        delta_x = (lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2)) * self.k
        delta_y = (lat2 - lat1) * self.k
        distance = math.sqrt(delta_x**2 + delta_y**2)
        return distance

if __name__ == '__main__':
    lat1 = -7.2676615
    lon1 = 112.7853807
    lat2 = -7.2676595
    lon2 = 112.7853833

    coord = Coordinate()
    distance = coord.calc_distance(lat1=lat1, lon1=lon1, lat2=lat2, lon2=lon2)

    print(f"{distance:.2f} meter")

