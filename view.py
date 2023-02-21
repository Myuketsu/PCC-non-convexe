import matplotlib.pyplot as plt
import geopandas as gpd
import networkx as nx
from shapely.geometry import LineString

def drawPath(path, pathLength, dfCountry, distance, countryName):
    '''Dessine le "path" sur le claque "dfCountry".'''
    minX, minY, _, _ = map(int, dfCountry.geometry.bounds.iloc[0])
    if isinstance(path, dict):
        path = list(path.keys())

    path_edges = list(zip(path, path[1:]))

    _, ax = plt.subplots()
    dfCountry.plot(ax=ax)

    plt.scatter(*path[0], color='red')
    plt.scatter(*path[-1], color='red')
    for edge in path_edges:
        line = gpd.GeoDataFrame(
            geometry=[LineString([edge[0], edge[1]])],
            crs=dfCountry.crs)
        line.plot(ax=ax, color='black')

    plt.title(f"Shortest path in {countryName}\nDistance : ~ {round(pathLength / 1000, 3)} km")
    plt.axis('off')
    plt.show()

def drawPathInGraph(path, pathLength, graph, dfCountry, countryName):
    '''Dessine le "path" dans le "graph" sur le calque "dfCountry".'''
    minX, minY, _, _ = map(int, dfCountry.geometry.bounds.iloc[0])
    if not isinstance(path, list):
        print("/!\\ Path must be a list /!\\")
        return 
    
    path_edges = list(zip(path, path[1:]))
    pos = {node: node for node in graph.nodes}

    _, ax = plt.subplots()
    dfCountry.plot(ax=ax)

    nx.draw(graph, pos=pos, ax=ax, node_color='black', node_size=5)
    nx.draw_networkx_nodes(graph, pos, nodelist=path, node_color='r', node_size=5)
    nx.draw_networkx_edges(graph, pos, edgelist=path_edges, edge_color='r', width=3)

    plt.title(f"Shortest path in {countryName}\nDistance : ~ {round(pathLength / 1000, 3)} km")
    plt.axis('off')
    plt.show()