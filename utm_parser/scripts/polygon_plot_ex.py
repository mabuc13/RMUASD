#!/usr/bin/env python

import numpy as np
import json
from matplotlib.patches import Circle, Wedge, Polygon
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
from utm import utmconv

msg = "[{\"valid_from_epoch\": \"1542814843\", \"name\": \"HCA Airport - Field 9\", \"geometry\": \"polygon\", \"valid_to_epoch\": \"1542815031\", \"coordinates\": \"10.32259,55.47212 10.32287,55.47221 10.32288,55.47249 10.32253,55.47265 10.32207,55.47250 10.32208,55.47223 10.32259,55.47212\", \"int_id\": \"9\"}, {\"valid_from_epoch\": \"1542814858\", \"name\": \"HCA Airport - Field 5\", \"geometry\": \"polygon\", \"valid_to_epoch\": \"1542815012\", \"coordinates\": \"10.32236,55.47204 10.32288,55.47157 10.32340,55.47175 10.32294,55.47218 10.32342,55.47233 10.32337,55.47237 10.32236,55.47204\", \"int_id\": \"5\"}, {\"valid_from_epoch\": \"1542814874\", \"name\": \"Modelflyveplads - Circle 1\", \"geometry\": \"circle\", \"valid_to_epoch\": \"1542815003\", \"coordinates\": \"10.41495,55.47172,20\", \"int_id\": \"22\"}, {\"valid_from_epoch\": \"1542814939\", \"name\": \"Modelflyveplads - Circle 4\", \"geometry\": \"circle\", \"valid_to_epoch\": \"1542815047\", \"coordinates\": \"10.41721,55.47204,40\", \"int_id\": \"25\"}]"

dict_json = {}
all_obj = json.loads(msg)

for json_obj in all_obj:
    dict_json[json_obj["int_id"]] = json_obj

def main():
    # Fixing random state for reproducibility
    np.random.seed(19680801)

    fig, ax = plt.subplots()

    patches = []

    utmcon = utmconv()

    for int_id, val in dict_json.items():
        coords = val["coordinates"]
        if val["geometry"] == "polygon":
            coords = coords.split(" ")
            i = 0
            for a in coords:
                coords[i] = a.split(',')
                i += 1
            new_c = []
            for i in coords:
                (hemisphere, zone, zlet, easting, northing) = utmcon.geodetic_to_utm(float(i[0]), float(i[1]))
                new_c.append([easting, northing])
            polygon = Polygon(new_c, True)
            patches.append(polygon)
            print(easting, northing)
        else:
            coords = coords.split(',')
            (hemisphere, zone, zlet, easting, northing) = utmcon.geodetic_to_utm(float(coords[0]), float(coords[1]))
            circle = Circle((easting, northing), float(coords[2]))
            #patches.append(circle)

    colors = 100*np.random.rand(len(patches))
    p = PatchCollection(patches, alpha=0.4)
    p.set_array(np.array(colors))
    ax.add_collection(p)

    plt.ylim(1141400, 1142400)
    plt.xlim(332400, 333400)
    fig.colorbar(p, ax=ax)

    plt.show()

if __name__ == "__main__":
    main()
