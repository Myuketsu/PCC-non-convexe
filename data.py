import os
os.environ['USE_PYGEOS'] = '0'

import math
import networkx as nx
import geopandas as gpd
from shapely.geometry import Point, Polygon, LineString
from tqdm import tqdm 

def generateSquareGraph(dfCountry, distance):
    '''Génère et retourne un graphe rectangulaire/carré de largeur "width" et hauteur "height".\n
    Chaque noeud est connecté au 8 noeuds dans chaque direction (sauf bordure).'''

    minX, minY, maxX, maxY = map(int, dfCountry.geometry.bounds.iloc[0])

    graph = nx.empty_graph(0)

    graph.add_nodes_from((i, j) for i in range(minX, maxX, distance) for j in range(minY, maxY, distance))

    # Génère un graphe sous forme de grille, chaque noeuds est connecté aux autres dans les 8 directions.
    graph.add_edges_from((
        (i, j), (i + distance, j), {'weight': distance}) 
            for i in range(minX, maxX - distance, distance)
                for j in range(minY, maxY, distance))
    graph.add_edges_from((
        (i, j), (i, j + distance), {'weight': distance})
            for i in range(minX, maxX, distance)
                for j in range(minY, maxY - distance, distance))
    
    diagWeight = int(math.sqrt(2 * distance * distance))
    graph.add_edges_from((
        (i, j), (i + distance, j + distance), {'weight': diagWeight})
            for i in range(minX, maxX - distance, distance)
                for j in range(minY, maxY - distance, distance))
    graph.add_edges_from((
        (i + distance, j), (i, j + distance), {'weight': diagWeight})
            for i in range(minX, maxX - distance, distance)
                for j in range(minY, maxY - distance, distance))
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
    minX, minY, _, _ = map(int, dfCountry.geometry.bounds.iloc[0])
    return ((int(x) // distance) * distance + minX % distance, (int(y) // distance) * distance + minY % distance)

def getGraph(dfCountry, distance, countryName=''):
    '''Retourne un graphe où chaque point est compris dans le "dfCountry".\n
    Sauvegarde le graphe à l'emplacement [./graph/nom.gexf] pour une prochaine lecture.'''
    if os.path.exists(f"./graph/{countryName.lower()}_{distance}.gexf"):
        return nx.read_gexf(f"./graph/{countryName.lower()}_{distance}.gexf", node_type=eval)
    
    minX, minY, maxX, maxY = map(int, dfCountry.geometry.bounds.iloc[0])

    # Génère une grille de points (géographiques) ou chaque point est à la même position d'un noeud du futur graphe.
    points = gpd.GeoDataFrame(
        {'geometry': 
            [Point(x, y)
                for y in tqdm(range(minY, maxY, distance), desc="Generation of points for the refinement of the graph")
                    for x in range(minX, maxX, distance)]
        }, crs=dfCountry.crs
    )
    
    # Génère un rectangle qui englobe tout le pays.
    square = gpd.GeoDataFrame(
        geometry=[Polygon([
            [minX, minY],
            [maxX, minY],
            [maxX, maxY],
            [minX, maxY]
        ])],
    crs=dfCountry.crs)

    # Récupère tout les points en dehors du pays.
    inverted_dfCountry = square.overlay(dfCountry, how='difference')
    pointsOutside = points.overlay(inverted_dfCountry, how='intersection')

    graph = generateSquareGraph(dfCountry, distance)

    for point in tqdm(pointsOutside['geometry'], desc="Removing nodes outside our dataframe"):
        graph.remove_node((point.x, point.y))

    # Créée des lignes (géographiques) représentant les arêtes du graph.
    edges = gpd.GeoDataFrame(
        {'geometry': 
            [LineString([*edge]) for edge in graph.edges]
        }, crs=dfCountry.crs
    )

    # Récupère toutes les lignes en dehors du pays.
    edges = gpd.sjoin(edges, inverted_dfCountry, how='inner', predicate='intersects')

    for edge in tqdm(edges['geometry'], desc="Removing edges outside our dataframe"):
        graph.remove_edge(edge.coords[0], edge.coords[-1])

    nodesToAdd = []
    edgesToAdd = []
    
    pointsInside = points.overlay(dfCountry, how='intersection')
    for boundary in tqdm(dfCountry.boundary.explode(index_parts=False).iloc[0].coords, desc="Refinement of the graph boundaries"):
        pointBoundary = Point(boundary)
        pointsInsideCircle = pointsInside.overlay(
            gpd.GeoDataFrame(geometry=[pointBoundary.buffer(distance)], crs=dfCountry.crs), 
            how='intersection')
        
        if not list(pointsInsideCircle['geometry']):
            continue
        
        nodesToAdd.append(boundary)
        for pointInside in pointsInsideCircle['geometry']:
            if not (dfCountry['geometry'].boundary).all().crosses(LineString([pointBoundary, pointInside])):
                edgesToAdd.append((boundary, (pointInside.x, pointInside.y), {'weight': pointBoundary.distance(pointInside)}))
    
    graph.add_nodes_from(nodesToAdd)
    graph.add_edges_from(edgesToAdd)

    # Enregistre le graphe.
    if (countryName != ''):
        nx.write_gexf(graph, f"./graph/{countryName.lower()}_{distance}.gexf")
    return graph

def shortest_path(graph, source, target, dfCountry, distance):
    '''Retourne le chemin le plus court dans le "graph".'''
    return nx.shortest_path(graph, 
        source=scaleCoordinate(*source, dfCountry, distance),
        target=scaleCoordinate(*target, dfCountry, distance),
    weight='weight')

def dichotomyOptimization(path, dfCountry, pbar=None):
    '''Optimise le "path" donné en argument en utilisant de manière dichotomique (rapide mais résultat moins "précis").\n
    Retourne le "path" optimisé sous forme de dict {point: distance par rapport au point précédent en mètre}.'''
    if pbar is None:
        pbar = tqdm(total = len(path) - 1, desc=f"Path optimization")
    pbar.update(1)

    indexNode = 0
    newPath, distPath = [path[0]], [0]
    while newPath[-1] != path[-1]:
        actualNode = path[indexNode]

        indexNode += 1
        pbar.update(1)

        nextNode = path[indexNode]
        dist = Point(*actualNode).distance(Point(*nextNode))
        dichotomyList = path[indexNode:]
        cptIndex = len(dichotomyList) // 2
        while dichotomyList:
            midIndex = len(dichotomyList) // 2
            currentNode = dichotomyList[midIndex]
            line = gpd.GeoDataFrame(
                geometry=[LineString([actualNode, currentNode])],
                crs=dfCountry.crs)

            # Intersection de la frontière avec la diagonale.
            if (dfCountry['geometry'].boundary).all().crosses(line['geometry'].iloc[0]):
                dichotomyList = dichotomyList[:midIndex]
                cptIndex -= midIndex // 2
                continue

            dichotomyList = dichotomyList[(midIndex + 1):]
            cptIndex += (midIndex + 1) // 2
            dist = line.boundary.iloc[0].geoms[0].distance(line.boundary.iloc[0].geoms[1])
            nextNode = currentNode
        pbar.update(cptIndex)
        indexNode += cptIndex
        distPath.append(dist)
        newPath.append(nextNode)

    return {node: dist for node, dist in zip(newPath, distPath)}
            
def exhaustiveOptimization(path, dfCountry, pbar=None):
    '''Optimise le "path" donné en argument en calculant toutes les combinaisons de manière exhaustive (résultat "précis" mais long à calculer).\n
    Retourne le "path" optimisé sous forme de dict {point: distance par rapport au point précédent en mètre}.'''
    if pbar is None:
        pbar = tqdm(total = len(path) - 1, desc="Path optimization")

    indexNode = 0
    newPath, distPath = [path[0]], [0]
    while newPath[-1] != path[-1]:
        actualNode = path[indexNode]

        indexNode += 1
        pbar.update(1)

        nextNode = path[indexNode]
        maxDistance = Point(*actualNode).distance(Point(*nextNode))
        indexNodeBeforeLoop = indexNode
        for subIndexNode, currentNode in enumerate(path[indexNode:]):
            line = gpd.GeoDataFrame(
                geometry=[LineString([actualNode, currentNode])],
                crs=dfCountry.crs)

            # Intersection de la frontière avec la diagonale.
            if (dfCountry['geometry'].boundary).all().crosses(line['geometry'].iloc[0]):
                continue
            
            # Calcule la distance euclidienne entre les deux extrémités de la diagonale.
            dist = line.boundary.iloc[0].geoms[0].distance(line.boundary.iloc[0].geoms[1])
            if maxDistance < dist:
                maxDistance = dist
                nextNode = currentNode
                indexNode = indexNodeBeforeLoop + subIndexNode
        pbar.update(indexNode - indexNodeBeforeLoop)
        distPath.append(maxDistance)
        newPath.append(nextNode)
    return {node: dist for node, dist in zip(newPath, distPath)}

def pathOptimization(path, dfCountry):
    '''Mixe l'optimisation dichotomique avec l'optimisation exhaustive pour un meilleur temps d'exécution et un résultat "précis".\n
    Retourne le "path" optimisé sous forme de dict {point: distance par rapport au point précédent en mètre}.'''
    pbar = tqdm(total = (len(path) - 1) * 2, desc="Path optimization")

    firstPath = dichotomyOptimization(path, dfCountry, pbar)
    pbar.total += (len(firstPath) - 1)
    pbar.refresh()
    firstPath = exhaustiveOptimization(list(firstPath.keys()), dfCountry, pbar)

    secondPath = dichotomyOptimization(path, dfCountry, pbar)
    pbar.total += (len(secondPath) - 1)
    pbar.refresh()
    secondPath = exhaustiveOptimization(list(secondPath.keys()), dfCountry, pbar)

    return min(firstPath, secondPath, key=getPathLength)

def getPathLength(path, graph=None):
    '''Retourne la longueur de "path" en mètre.'''
    if isinstance(path, list):
        pathEdges = list(zip(path, path[1:]))
        return sum([graph.get_edge_data(*edge)['weight'] for edge in pathEdges])
    return sum(path.values())