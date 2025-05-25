import math

class Coordinate:
    
    def __init__ (self, lat1: float, lon1: float, lat2: float, lon2: float):
        self.k = 113200
        self.lat1 = lat1
        self.lon1 = lon1
        self.lat2 = lat2
        self.lon2 = lon2
        self.distance = self.calc_distance()

    def calc_distance(self):
        delta_x = (self.lon2 - self.lon1) * math.cos(math.radians((self.lat1 + self.lat2) / 2)) * self.k
        delta_y = (self.lat2 - self.lat1) * self.k
        distance = math.sqrt(delta_x**2 + delta_y**2)
        return distance

if __name__ == '__main__':
    lat1 = -7.2676615
    lon1 = 112.7853807
    lat2 = -7.2676595
    lon2 = 112.7853833

    coord = Coordinate(lat1=lat1, lon1=lon1, lat2=lat2, lon2=lon2)
    distance = coord.distance

    print(f"{distance:.2f} meter")

