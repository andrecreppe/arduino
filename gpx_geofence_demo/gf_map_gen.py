import os
import webbrowser
import folium
from pyproj import Transformer
from shapely.geometry import Polygon

# ----- TOGGLE VARIABLES -----
SATELLITE = True       # True = Satellite basemap; False = Normal (OpenStreetMap)
SHOW_LABELS = True     # Only applies when SATELLITE = True
# -----------------------

# Runway polygon (lat, lon)
# Embraer GPX <> Runway 02-20
# runway = [
#     (-21.750669, -48.406295),  # NW
#     (-21.750669, -48.404549),  # NE
#     (-21.796963, -48.403810),  # SE
#     (-21.796989, -48.405407),  # SW
# ]
# WARNING_RADIUS = 90
# Embraer GPX <> Alpha Yard
runway = [
    (-21.762005, -48.403078),  # NW
    (-21.761996, -48.402081),  # NE
    (-21.763241, -48.402057),  # SE
    (-21.763264, -48.403050),  # SW
]
WARNING_RADIUS = 35

# Projections
transformer_to_m = Transformer.from_crs("EPSG:4326", "EPSG:31983", always_xy=True)
transformer_to_latlon = Transformer.from_crs("EPSG:31983", "EPSG:4326", always_xy=True)

# Helpers
def to_meters(lat, lon):
    # always_xy=True -> inputs/outputs are (lon, lat)
    x, y = transformer_to_m.transform(lon, lat)
    return x, y

def to_latlon(x, y):
    lon, lat = transformer_to_latlon.transform(x, y)
    return lat, lon

# Convert runway to shapely polygon (in meters)
runway_m = Polygon([to_meters(lat, lon) for lat, lon in runway])

# Buffer to create warning zone (meters)
warning_m = runway_m.buffer(WARNING_RADIUS)

# Convert back to lat/lon
warning = [to_latlon(x, y) for x, y in warning_m.exterior.coords]

# Map center
center_lat = sum(p[0] for p in runway) / len(runway)
center_lon = sum(p[1] for p in runway) / len(runway)

# Create map depending on the SATELLITE flag
if SATELLITE:
    # Start without a default base layer
    m = folium.Map(location=[center_lat, center_lon], zoom_start=16, tiles=None)

    # Satellite basemap: Esri World Imagery
    folium.TileLayer(
        tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
        attr='Tiles © Esri — Sources: Esri, Maxar, Earthstar Geographics, and the GIS User Community',
        name='Esri World Imagery',
        overlay=False,
        control=True
    ).add_to(m)

    # Optional: labels overlays to improve readability on satellite imagery
    if SHOW_LABELS:
        folium.TileLayer(
            tiles='https://server.arcgisonline.com/ArcGIS/rest/services/Reference/World_Boundaries_and_Places/MapServer/tile/{z}/{y}/{x}',
            attr='© Esri',
            name='Boundaries & Places (labels)',
            overlay=True,
            control=True
        ).add_to(m)

        folium.TileLayer(
            tiles='https://server.arcgisonline.com/ArcGIS/rest/services/Reference/World_Transportation/MapServer/tile/{z}/{y}/{x}',
            attr='© Esri',
            name='Transportation (labels)',
            overlay=True,
            control=True
        ).add_to(m)

    folium.LayerControl().add_to(m)

else:
    # Normal OpenStreetMap basemap
    m = folium.Map(location=[center_lat, center_lon], zoom_start=16, tiles='OpenStreetMap', control_scale=True)

# Add warning polygon (orange)
folium.Polygon(
    warning,
    color="orange",
    fill=True,
    fill_opacity=0.2,
    popup=f"{WARNING_RADIUS}m Warning Zone"
).add_to(m)

# Add runway exclusion zone polygon (red)
folium.Polygon(
    runway,
    color="red",
    fill=True,
    fill_opacity=0.4,
    popup="Runway Exclusion Zone"
).add_to(m)

# Generate HTML file
filename = "gf_map.html"
m.save(filename)

# Open after saving
filepath = os.path.join(os.getcwd(), filename)
webbrowser.open_new_tab(f'file:///{filepath}')
