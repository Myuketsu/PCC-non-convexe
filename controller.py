import data
import view

'''
/!\ LIBRARIES REQUISES /!\
    - geopandas
    - matplotlib
    - networkx
    - pyproj
    - tqdm
'''

from pyproj import Transformer
from time import time

start_time = time()

crsCroatia = 3765
dfCroatia = data.read_file('./shapefile/Croatia_Shapefile.zip', crsCroatia)
dfCroatia = (dfCroatia[['NAME_0', 'geometry']]
                .dissolve(by='NAME_0', aggfunc='sum')['geometry']
                .explode(index_parts=False)).simplify(500)
dfCroatia = data.deleteIslands(dfCroatia)

# /!\ Attention /!\ : Moins de 250 augmente drastiquement le temps d'exécution et le poids du graphe (pour 300 : poids -> ~250MB).
distance = 1000

graphCroatia = data.getGraph(dfCroatia, distance, "Croatia")

transformer = Transformer.from_crs(4326, crsCroatia)

Zagreb = transformer.transform(45.81485300993664, 15.981875537751817) # (459504, 5075320)
Split = transformer.transform(43.51472042099229, 16.44360710780893) # (495406, 4819404)
Osijek = transformer.transform(45.59623554382566, 18.698120902173297) # (671780, 5051020)
Pula = transformer.transform(44.86624294403039, 13.849061868993267) # (290531, 4972931)
Zadar = transformer.transform(44.11935404887227, 15.231459924256162)

path = data.shortest_path(graphCroatia, Split, Zagreb, dfCroatia, distance)
path = data.pathOptimization(path, dfCroatia)

pathLength = data.getPathLength(path, graphCroatia)

print(f"---Distance found in {round(time() - start_time, 3)} seconds---")

print(f"-> Distance : ~ {round(pathLength / 1000, 3)} km")

view.drawPath(path, pathLength, dfCroatia, distance, "Croatia")
# view.drawPathInGraph(path, pathLength, graphCroatia, dfCroatia, "Croatia")