import folium
from pyproj import Transformer
from shapely.geometry import Polygon

# Runway polygon (lat, lon)
# GPX
runway = [
    (-21.751308, -48.405645),  # NW
    (-21.751315, -48.405206),  # NE
    (-21.796440, -48.404396),  # SE
    (-21.796451, -48.404831),  # SW
]
# USP SÃO CARLOS
# runway = [
#     (-22.002662, -47.900222),  # NW
#     (-22.002654, -47.896139),  # NE
#     (-22.011453, -47.895683),  # SE
#     (-22.011480, -47.899861),  # SW
# ]

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
warning_m = runway_m.buffer(150)

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
               popup="150m Warning Zone").add_to(m)

m.save("geofence_map.html")
print(">> Map saved as geofence_map.html — open it in your browser.")
