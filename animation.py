import os
os.environ['USE_PYGEOS'] = '0'

from tqdm import tqdm 
import geopandas as gpd
from shapely.geometry import Point
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import data

#---Gestion des données---
dfCountry = data.read_file('./shapefile/Croatia_Shapefile.zip', 3765)
dfCountry = (dfCountry[['NAME_0', 'geometry']]
                .dissolve(by='NAME_0', aggfunc='sum')['geometry']
                .explode(index_parts=False)).simplify(500)
dfCountry = data.deleteIslands(dfCountry)

minX, minY, maxX, maxY = map(int, dfCountry.geometry.bounds.iloc[0])

#---Création des points aléatoires compris dans le pays en question---
nbr_points = 5_000
randomPoints = gpd.GeoDataFrame(geometry=[Point(random.randint(minX, maxX), random.randint(minY, maxY)) for _ in range(nbr_points)], crs=dfCountry.crs)
pointsWithinCountry = gpd.sjoin(randomPoints, dfCountry, predicate='within')

#---Affichage sous forme d'un point de la moyenne de tous les points---
nbrTotalPoints = len(pointsWithinCountry.index)
x = sum(pointsWithinCountry.geometry.x)
y = sum(pointsWithinCountry.geometry.y)

fig, ax = plt.subplots()
dfCountry.plot(ax=ax)
gpd.GeoDataFrame(geometry=[Point(x / nbrTotalPoints, y / nbrTotalPoints)]).plot(ax=ax, edgecolor='blue')
title = ax.text(0.5, 1.05, "Moyenne des points dans le pays.", 
                bbox={'facecolor': 'mediumslateblue', 'alpha': 0.75}, 
                transform=ax.transAxes, ha="center")

#---Animation---
if (True):
    pbar = tqdm(total = nbrTotalPoints - 1, desc="Placement of the points")
    backgroundPoints = [ax.plot([], [], marker="o", markersize=5, markerfacecolor="magenta") for _ in range(50)]
    averagePoint, = ax.plot([], [], marker="o", markersize=10, markerfacecolor="red")
    def plotPoint(i):
        backgroundPoints[i % 50][0].set_data(pointsWithinCountry.geometry.iloc[i].x, pointsWithinCountry.geometry.iloc[i].y)
        averagePoint.set_data(sum(pointsWithinCountry.geometry.iloc[:(i + 1)].x) / (i + 1), sum(pointsWithinCountry.geometry.iloc[:(i + 1)].y) / (i + 1))
        title.set_text(f"Moyenne des points dans le pays.\nPoints posés : {i} / {nbrTotalPoints - 1}")
        pbar.update(1)

    anim = animation.FuncAnimation(fig, func=plotPoint, frames=range(nbrTotalPoints), interval=10, repeat=False)

#---Affichage de la fenêtre---
plt.axis('off')
plt.show()