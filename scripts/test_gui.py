# https://nicegui.io/documentation/leaflet
# https://leaflet-extras.github.io/leaflet-providers/preview/

# m = ui.leaflet(center=(39.916668, 116.383331))

from nicegui import ui

m = ui.leaflet(center=(39.916668, 116.383331), zoom=3)
m.clear_layers()
m.tile_layer(
    url_template=r'https://tiles.stadiamaps.com/tiles/alidade_satellite/{z}/{x}/{y}{r}.{ext}',
    options={
        'minZoom': 0,
        'maxZoom': 20,
        'attribution':
            '&copy; CNES, Distribution Airbus DS, © Airbus DS, © PlanetObserver (Contains Copernicus Data) | &copy; <a href="https://www.stadiamaps.com/" target="_blank">Stadia Maps</a> &copy; <a href="https://openmaptiles.org/" target="_blank">OpenMapTiles</a> &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
        'ext': 'jpg'
    },
)

ui.run()