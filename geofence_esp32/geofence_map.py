import os
import webbrowser
import folium
from pyproj import Transformer
from shapely.geometry import Polygon

# WARNING_RADIUS = 150
WARNING_RADIUS = 35

# Runway polygon (lat, lon)
# Embraer GPX <> Runway 02-20
# runway = [
#     (-21.751308, -48.405645),  # NW
#     (-21.751315, -48.405206),  # NE
#     (-21.796440, -48.404396),  # SE
#     (-21.796451, -48.404831),  # SW
# ]
# Embraer GPX <> Alpha Yard
runway = [
    (-21.762005, -48.403078),  # NW
    (-21.761996, -48.402081),  # NE
    (-21.763241, -48.402057),  # SE
    (-21.763264, -48.403050),  # SW
]

# Projections
transformer_to_m = Transformer.from_crs("EPSG:4326", "EPSG:31983", always_xy=True)  
transformer_to_latlon = Transformer.from_crs("EPSG:31983", "EPSG:4326", always_xy=True)

# Helpers
def to_meters(lat, lon):
    x, y = transformer_to_m.transform(lon, lat)
    return x, y

def to_latlon(x, y):
    lon, lat = transformer_to_latlon.transform(x, y)
    return lat, lon

# Convert runway to shapely polygon (in meters)
runway_m = Polygon([to_meters(lat, lon) for lat, lon in runway])

# Buffer by 150 m to create warning zone
warning_m = runway_m.buffer(WARNING_RADIUS)

# Convert back to lat/lon
warning = [to_latlon(x, y) for x, y in warning_m.exterior.coords]

# Map center
center_lat = sum(p[0] for p in runway) / len(runway)
center_lon = sum(p[1] for p in runway) / len(runway)
m = folium.Map(location=[center_lat, center_lon], zoom_start=16)

# Add runway polygon (red)
folium.Polygon(runway, color="red", fill=True, fill_opacity=0.4,
               popup="Runway Exclusion Zone").add_to(m)

# Add warning polygon (orange)
folium.Polygon(warning, color="orange", fill=True, fill_opacity=0.2,
               popup=f"{WARNING_RADIUS}m Warning Zone").add_to(m)

# Generate HTML file
filename = "geofence_map.html"
m.save(filename)

# Open after saving
filepath = os.path.join(os.getcwd(), filename)

webbrowser.open_new_tab(f'file:///{filepath}')
