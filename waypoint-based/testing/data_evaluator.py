from math import sqrt
import numpy as np
import csv
from geopy import distance

def get_lat_long_distance(lat1,lon1,lat2,lon2):
    return distance.distance((lat1,lon1),(lat2,lon2))

def get_3d_distance(lat1,lon1,alt1,lat2,lon2,alt2):
    lat_lon_dist = get_lat_long_distance(lat1,lon1,lat2,lon2).km
    alt_dist = alt2-alt1
    return sqrt(lat_lon_dist**2 + alt_dist**2)

def evaluate(filename):
    file_csv = open(filename)
    data_csv = csv.reader(file_csv)
    list_csv = list(data_csv)[1:]
    distances = []
    for line in list_csv:
        lat1 = float(line[0])
        lat2 = float(line[1])
        lon1 = float(line[2])
        lon2 = float(line[3])
        alt1 = float(line[4])
        alt2 = float(line[5])
        d = get_3d_distance(lat1,lon1,alt1,lat2,lon2,alt2)
        distances.append(d)
    
    errors = [2-x for x in distances]
    rmse = sqrt(np.average([x**2 for x in errors]))
    return rmse

filenames = ["zz_noiseless_2.csv","zz_noisy_2.csv","zz_noiseless_10.csv","zz_noisy_10.csv","circle_noiseless_1.csv","circle_noisy_1.csv","circle_noiseless_5.csv","circle_noisy_5.csv"]

if __name__ == '__main__':
    for x in filenames:
        rmse = evaluate("csv/"+x)
        print(x + ": " + str(rmse))
    