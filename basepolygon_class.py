import osmnx as ox

from shapely.geometry import Polygon


class GoalPolygon:

    def __init__(self, osm_name, **kwargs):
        self.name = None
        self.osm_name = osm_name
        self.buffer_dict = 500
        self.network_type = 'all'
        self.frame = ox.geocode_to_gdf(self.osm_name, buffer_dist=self.buffer_dict)
        self._graph = ox.graph_from_place(self.osm_name, network_type=self.network_type, buffer_dist=self.buffer_dict)
        self.frameproj = ox.project_gdf(ox.geocode_to_gdf(self.osm_name))
        self.polygon = self.coordinates_poly()
        self.polygon_coords = self.get_polygon_coords()

    @property
    def graph(self):
        return self._graph

    @graph.setter
    def graph(self, val):
        self._graph = ox.graph_from_place(self.osm_name, **val)

    def coordinates_poly(self):
        coordinates = []
        polygon = self.frameproj.boundary[0]
        # coordinates = list(polygon.coords)
        for i in polygon.coords:
            coordinates.append(list(i))
        return Polygon(coordinates)

    def network_clean(self,
                      key='all',
                      num=1000
                      ):

        network = ox.graph_from_place(self.osm_name,
                                      network_type=key,
                                      buffer_dist=num)

        network = ox.project_graph(network)
        gdf_nodes, gdf_edges = ox.graph_to_gdfs(network)

        for i in gdf_edges.geometry.keys():
            if gdf_edges.geometry[i].crosses(self.polygon) or self.polygon.covers(gdf_edges.geometry[i]):
                network.remove_edge(*i)
            else:
                pass
        network = ox.utils_graph.remove_isolated_nodes(network)

        nodes, edges = ox.graph_to_gdfs(network)

        setattr(self, 'exterior_graph', network)
        return network, edges, nodes

    def network_edges_coords(self, **kwargs):
        nn_coords = []
        network, edges, nodes = self.network_clean(**kwargs)
        for n in edges.geometry:
            try:
                iterator = iter(n)
                for ii in range(len(n)):
                    nn_coords.append(list(n[ii].coords))
            except TypeError:
                nn_coords.append(list(n.coords))
        return nn_coords

    def get_osmid(self, **kwargs):
        network, edges, nodes = self.network_clean(**kwargs)
        ed_id = list(edges.geometry.keys())
        n_id = list(nodes.geometry.keys())
        return ed_id, n_id

    def get_network_edges_coords(self, **kwargs):
        node_coords = []
        network, edges, nodes = self.network_clean(**kwargs)
        for n in nodes.geometry:
            node_coords.append(list(*n.coords))
        return node_coords

    def get_polygon_coords(self):
        polygon = self.frameproj.boundary[0]
        coordinates = list(polygon.coords)
        return coordinates

    def graph_to_gdfs(self):
        nodes, edges = ox.graph_to_gdfs(self.network)
        setattr(self, 'nodes', nodes)
        setattr(self, 'edges', edges)
        return nodes, edges
