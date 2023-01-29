import os
os.environ['USE_PYGEOS'] = '0'

import math
import networkx as nx
import geopandas as gpd
from shapely.geometry import Point, Polygon, LineString
from tqdm import tqdm 

def generateSquareGraph(width, height, distance):
    '''Génère et retourne un graphe rectangulaire/carré de largeur "width" et hauteur "height".\n
    Chaque noeud est connecté au 8 noeuds dans chaque direction (sauf bordure).'''
    graph = nx.grid_2d_graph(width, height)
    nx.set_edge_attributes(graph, values=distance, name='weight')

    graph.add_edges_from([
        ((x, y), (x + 1, y + 1))
        for x in range(width - 1)
        for y in range(height - 1)
    ] + [
        ((x + 1, y), (x, y + 1))
        for x in range(width - 1)
        for y in range(height - 1)
    ], weight=int(math.sqrt(2 * distance * distance)))
    return graph

def read_file(filename, epsg):
    '''Ouvre le fichier "filename" pour le convertir en geoDataFrame avec son "epsg" correspondant.'''
    return gpd.read_file(filename).to_crs(epsg=epsg)

def deleteIslands(dfCountry):
    '''Supprime tous les polygones qui ne sont pas reliés directement au plus gros polygone.'''
    polys = (gpd.GeoSeries(dfCountry).reset_index(drop=True))
    dfCountry = (
        gpd.GeoDataFrame(data={'area': polys.area}, geometry=polys, crs=polys.crs)
        .sort_values("area", ascending=0)
        .head(1)
        .reset_index(drop=True)
    )
    return dfCountry

def scaleCoordinate(x, y, dfCountry, distance):
    '''Transforme et retourne les coordonnées en CRS actuel en l'equivalent dans le graphe.'''
    minX, minY, maxX, maxY = map(int, dfCountry.geometry.bounds.iloc[0])
    return ((x - minX) // distance, (y - minY) // distance)

def getGraph(dfCountry, distance, countryName):
    '''Retourne un graphe où chaque point est compris dans le "dfCountry".\n
    Sauvegarde le graphe à l'emplacement [./graph/nom.gexf] pour une prochaine lecture.'''
    if os.path.exists(f"./graph/{countryName.lower()}_{distance}.gexf"):
        return nx.read_gexf(f"./graph/{countryName.lower()}_{distance}.gexf", node_type=eval)
    
    minX, minY, maxX, maxY = map(int, dfCountry.geometry.bounds.iloc[0])
    width, height = ((maxX - minX + 1) // distance, (maxY - minY + 1) // distance)

    points = gpd.GeoDataFrame(
        {'geometry': 
            [Point(minX + x * distance, minY + y * distance)
                for y in tqdm(range(height), desc="Generation of points for the refinement of the graph")
                    for x in range(width)]
        }, crs=dfCountry.crs
    )

    square = gpd.GeoDataFrame(
        geometry=[Polygon([
            [minX, minY],
            [maxX, minY],
            [maxX, maxY],
            [minX, maxY]
        ])],
    crs=dfCountry.crs)

    inverted_dfCountry = square.overlay(dfCountry, how='difference')
    points = points.overlay(inverted_dfCountry, how='intersection')

    graph = generateSquareGraph(width, height, distance)

    for point in tqdm(points['geometry'], desc="Removing nodes outside our dataframe"):
        graph.remove_node(((point.x - minX) // distance, (point.y - minY) // distance))

    nx.write_gexf(graph, f"./graph/{countryName.lower()}_{distance}.gexf")
    return graph

def shortest_path(graph, source, target, dfCountry, distance):
    '''Retourne le chemin le plus court dans le "graph".'''
    return nx.shortest_path(graph, 
        source=scaleCoordinate(*source, dfCountry, distance),
        target=scaleCoordinate(*target, dfCountry, distance),
    weight='weight')

def pathOptimization(path, dfCountry, distance):
    '''Optimise le "path" donné en argument.\n
    Retourne le "path" optimisé sous forme de dict {point: distance par rapport au point précédent en mètre}.'''
    pbar = tqdm(total = len(path) - 1, desc="Path optimization")

    minX, minY, maxX, maxY = map(int, dfCountry.geometry.bounds.iloc[0])
    dfCountry['geometry'] = dfCountry['geometry'].boundary

    indexNode = 0
    newPath, distPath = [path[0]], [0]
    while newPath[-1] != path[-1]:
        actualNode = path[indexNode]

        indexNode += 1
        pbar.update(1)

        maxDistance = 0
        nextNode = path[indexNode]
        actualIndexNode = indexNode
        for subIndexNode, currentNode in enumerate(path[indexNode:]):
            line = gpd.GeoDataFrame(
                geometry=[LineString([
                    (minX + actualNode[0] * distance, minY + actualNode[1] * distance),
                    (minX + currentNode[0] * distance, minY + currentNode[1] * distance)])],
                crs=dfCountry.crs)

            join = gpd.sjoin(line, dfCountry, predicate='intersects')
            if list(join.geometry):
                continue
            
            dist = line.boundary.iloc[0].geoms[0].distance(line.boundary.iloc[0].geoms[1])
            if maxDistance < dist:
                maxDistance = dist
                nextNode = currentNode
                indexNode = actualIndexNode + subIndexNode
        pbar.update(indexNode - actualIndexNode)
        distPath.append(maxDistance)
        newPath.append(nextNode)
    return {node: dist for node, dist in zip(newPath, distPath)}

def getPathLength(path, graph):
    '''Retourne la longueur de "path" en mètre.'''
    if isinstance(path, list):
        pathEdges = list(zip(path, path[1:]))
        return sum([graph.get_edge_data(*edge)['weight'] for edge in pathEdges])
    return sum(path.values())