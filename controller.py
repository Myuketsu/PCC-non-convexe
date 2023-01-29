import data
import view
from time import time

start_time = time()

dfCroatia = data.read_file('./shapefile/Croatia_Shapefile.zip', 3765)
dfCroatia = (dfCroatia[['NAME_0', 'geometry']]
                .dissolve(by='NAME_0', aggfunc='sum')['geometry']
                .explode(index_parts=False)).simplify(500)
dfCroatia = data.deleteIslands(dfCroatia)

#/!\ Attention /!\ : Moins de 250 augmente drastiquement le temps d'execution et le poids du graphe (pour 250 : poids -> ~300MB).
distance = 1000

graphCroatia = data.getGraph(dfCroatia, distance, "Croatia")

Zagreb = (459504, 5075320)
Split = (495406, 4819404)
Osijek = (671780, 5051020)
Pula = (290531, 4972931)

path = data.shortest_path(graphCroatia, Split, Zagreb, dfCroatia, distance)
path = data.pathOptimization(path, dfCroatia, distance)

pathLength = data.getPathLength(path, graphCroatia)

print(f"---Distance found in {round(time() - start_time, 3)} seconds---")

print(f"-> Distance : ~ {round(pathLength / 1000, 3)} km")

view.drawPath(path, pathLength, dfCroatia, distance)
#view.drawPathInGraph(path, pathLength, graphCroatia, dfCroatia, distance)