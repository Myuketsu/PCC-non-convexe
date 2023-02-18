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

    # Génère une grille sur forme de graphe.
    graph = nx.grid_2d_graph(width, height)
    nx.set_edge_attributes(graph, values=distance, name='weight')

    # Ajoute au graphe les arêtes en diagonale.
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
    minX, minY, _, _ = map(int, dfCountry.geometry.bounds.iloc[0])
    return ((x - minX) // distance, (y - minY) // distance)

def getGraph(dfCountry, distance, countryName=''):
    '''Retourne un graphe où chaque point est compris dans le "dfCountry".\n
    Sauvegarde le graphe à l'emplacement [./graph/nom.gexf] pour une prochaine lecture.'''
    if os.path.exists(f"./graph/{countryName.lower()}_{distance}.gexf"):
        return nx.read_gexf(f"./graph/{countryName.lower()}_{distance}.gexf", node_type=eval)
    
    minX, minY, maxX, maxY = map(int, dfCountry.geometry.bounds.iloc[0])
    width, height = ((maxX - minX + 1) // distance, (maxY - minY + 1) // distance)

    # Génère une grille de points (géographiques) ou chaque point est à la même position d'un noeud du futur graphe.
    points = gpd.GeoDataFrame(
        {'geometry': 
            [Point(minX + x * distance, minY + y * distance)
                for y in tqdm(range(height), desc="Generation of points for the refinement of the graph")
                    for x in range(width)]
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
    points = points.overlay(inverted_dfCountry, how='intersection')

    graph = generateSquareGraph(width, height, distance)

    for point in tqdm(points['geometry'], desc="Removing nodes outside our dataframe"):
        graph.remove_node(((point.x - minX) // distance, (point.y - minY) // distance))
    
    # Récupère toutes les arêtes du graph et les transferts dans le bon CRS.
    edgesToCRS = [((minX + edge[0][0] * distance, minY + edge[0][1] * distance), 
                  (minX + edge[1][0] * distance, minY + edge[1][1] * distance))
                    for edge in graph.edges]
    
    # Créée des lignes représentant les arêtes du graph.
    edges = gpd.GeoDataFrame(
        {'geometry': 
            [LineString([*edge]) for edge in edgesToCRS]
        }, crs=dfCountry.crs
    )

    # Récupère toutes les lignes en dehors du pays.
    edges = gpd.sjoin(edges, inverted_dfCountry, how='inner', predicate='intersects')

    for edge in tqdm(edges['geometry'], desc="Removing edges outside our dataframe"):
        edge = (edge.coords[0], edge.coords[-1])
        graph.remove_edge(((edge[0][0] - minX) // distance, (edge[0][1] - minY) // distance),
                          ((edge[1][0] - minX) // distance, (edge[1][1] - minY) // distance))
    
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

def dichotomyOptimization(path, dfCountry, distance, pbar=None):
    '''Optimise le "path" donné en argument en utilisant de manière dichotomique (rapide mais résultat moins "précis").\n
    Retourne le "path" optimisé sous forme de dict {point: distance par rapport au point précédent en mètre}.'''
    if pbar is None:
        pbar = tqdm(total = len(path) - 1, desc=f"Path optimization")
    pbar.update(1)

    # On récupère les frontières du pays pour faire des intersections par la suite avec diagonales testées.
    dfCopyCountry = dfCountry.copy(deep=True)
    minX, minY, _, _ = map(int, dfCopyCountry.geometry.bounds.iloc[0])
    dfCopyCountry['geometry'] = dfCopyCountry['geometry'].boundary

    indexNode = 0
    newPath, distPath = [path[0]], [0]
    while newPath[-1] != path[-1]:
        actualNode = path[indexNode]

        indexNode += 1
        pbar.update(1)

        nextNode = path[indexNode]
        dist = Point((minX + actualNode[0] * distance, minY + actualNode[1] * distance))\
            .distance(Point((minX + nextNode[0] * distance, minY + nextNode[1] * distance)))
        dichotomyList = path[indexNode:]
        cptIndex = len(dichotomyList) // 2
        while dichotomyList:
            midIndex = len(dichotomyList) // 2
            currentNode = dichotomyList[midIndex]
            line = gpd.GeoDataFrame(
                geometry=[LineString([
                    (minX + actualNode[0] * distance, minY + actualNode[1] * distance),
                    (minX + currentNode[0] * distance, minY + currentNode[1] * distance)])],
                crs=dfCopyCountry.crs)

            # Intersection de la frontière avec la diagonale.
            if dfCopyCountry['geometry'].all().intersects(line['geometry'].iloc[0]):
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
            
def exhaustiveOptimization(path, dfCountry, distance, pbar=None):
    '''Optimise le "path" donné en argument en calculant toutes les combinaisons de manière exhaustive (résultat "précis" mais long à calculer).\n
    Retourne le "path" optimisé sous forme de dict {point: distance par rapport au point précédent en mètre}.'''
    if pbar is None:
        pbar = tqdm(total = len(path) - 1, desc="Path optimization")

    # On récupère les frontières du pays pour faire des intersections par la suite avec diagonales testées.
    dfCopyCountry = dfCountry.copy(deep=True)
    minX, minY, _, _ = map(int, dfCopyCountry.geometry.bounds.iloc[0])
    dfCopyCountry['geometry'] = dfCopyCountry['geometry'].boundary

    indexNode = 0
    newPath, distPath = [path[0]], [0]
    while newPath[-1] != path[-1]:
        actualNode = path[indexNode]

        indexNode += 1
        pbar.update(1)

        nextNode = path[indexNode]
        maxDistance = Point((minX + actualNode[0] * distance, minY + actualNode[1] * distance))\
            .distance(Point((minX + nextNode[0] * distance, minY + nextNode[1] * distance)))
        indexNodeBeforeLoop = indexNode
        for subIndexNode, currentNode in enumerate(path[indexNode:]):
            line = gpd.GeoDataFrame(
                geometry=[LineString([
                    (minX + actualNode[0] * distance, minY + actualNode[1] * distance),
                    (minX + currentNode[0] * distance, minY + currentNode[1] * distance)])],
                crs=dfCopyCountry.crs)

            # Intersection de la frontière avec la diagonale.
            if dfCopyCountry['geometry'].all().intersects(line['geometry'].iloc[0]):
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

def pathOptimization(path, dfCountry, distance):
    '''Mixe l'optimisation dichotomique avec l'optimisation exhaustive pour un meilleur temps d'exécution et un résultat "précis".\n
    Retourne le "path" optimisé sous forme de dict {point: distance par rapport au point précédent en mètre}.'''
    pbar = tqdm(total = (len(path) - 1) * 2, desc="Path optimization")

    firstPath = dichotomyOptimization(path, dfCountry, distance, pbar)
    pbar.total += (len(firstPath) - 1)
    pbar.refresh()
    firstPath = exhaustiveOptimization(list(firstPath.keys()), dfCountry, distance, pbar)

    secondPath = dichotomyOptimization(path, dfCountry, distance, pbar)
    pbar.total += (len(secondPath) - 1)
    pbar.refresh()
    secondPath = exhaustiveOptimization(list(secondPath.keys()), dfCountry, distance, pbar)

    return min(firstPath, secondPath, key=getPathLength)

def getPathLength(path, graph=None):
    '''Retourne la longueur de "path" en mètre.'''
    if isinstance(path, list):
        pathEdges = list(zip(path, path[1:]))
        return sum([graph.get_edge_data(*edge)['weight'] for edge in pathEdges])
    return sum(path.values())