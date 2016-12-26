#Shashank Ojha
#Romania Railroad
from pickle import dump
from pickle import load
from time import time
from math import * # pi , acos , sin , cos
from heapq  import *

"""
+--------------------------+
|                          |
|      Shashank Ojha       |
|        Period 1          |
|                          |
+--------------------------+
"""

# Arad
# Bucharest
# Craiova
# Drobeta
# Eforie
# Fagaras
# Giurgiu
# Harsova
# Iasi
# Lugoj
# Mehadia
# Neamt
# Oradea
# Pitesti
# Ramnicu Valcea
# Sibiu
# Timisoara
# Urziceni
# Vaslui
# Zerind

start = "Bucharest"
target = "Arad"
cities = {};

def findPathAStar(NeighborsDictionary, locations, start, target): # need to add arguments to all the headers
    fCost = 0
    qCount = 0
    visited = []
    upcomingHeap = []
    upcoming = []
    heappush(upcomingHeap, (0, start))
    parent = {}
    gCost = { start : 0 }
    count = 0;
    while(upcomingHeap): #just A* search- does not build path
        # print upcomingHeap
        qCount+=1;
        x, city = heappop(upcomingHeap)
        visited.append(city)
        # print city
        if(city == target): #and city not in upcoming):
             break;
        for neighbor in NeighborsDictionary[city]:
            if(neighbor not in visited and ( neighbor not in upcoming or gCost[neighbor] >  gCost[city] + calcDistance(neighbor,city, locations ) ) ):
                gCost[neighbor] = gCost[city] + calcDistance(neighbor, city, locations )
                fCost =  gCost[neighbor] + findHearistic(neighbor, target, locations)  #qCount
                heappush(upcomingHeap, (fCost, neighbor))
                parent[neighbor] = [city,  calcDistance(neighbor, city, locations )]
                upcoming.append(neighbor)
    print ("qCount = "),;
    print (qCount);
    return parent
def findPathUniformCost(NeighborsDictionary, locations, start, target): # need to add arguments to all the headers
    fCost = 0
    qCount = 0
    visited = []
    upcoming = []
    upcomingHeap = []
    heappush(upcomingHeap, (0, start))
    parent = {}
    gCost = { start : 0 }
    count = 0;
    while(upcomingHeap): #just A* search- does not build path
        # print upcomingHeap
        qCount+=1;
        x, city = heappop(upcomingHeap)
        visited.append(city)
        # print 'current city =', city
        if(city == target and city not in upcomingHeap):
             break;
        for neighbor in NeighborsDictionary[city]:
            if(neighbor not in visited and ( neighbor not in upcoming or (gCost[neighbor] > gCost[city] + calcDistance(neighbor,city, locations )) ) ):
                gCost[neighbor] = gCost[city] + calcDistance(neighbor, city, locations )
                fCost =  gCost[neighbor]
                heappush(upcomingHeap, (fCost, neighbor))
                parent[neighbor] =  [city,  calcDistance(neighbor, city, locations )]
                upcoming.append(neighbor)
    print ("qCount = "),;
    print (qCount);
    return parent
def findHearistic(cityOne, cityTwo, locations):
    return calcDistance(cityOne, cityTwo, locations);

def calcDistance(cityOne, cityTwo, locations):
   #
   y1 = float((locations[cityOne])[0])
   x1 = float((locations[cityOne])[1])

   y2 = float((locations[cityTwo])[0])
   x2 = float((locations[cityTwo])[1])
   #
   R   = 3958.76 # miles
   #
   y1 *= pi/180.0
   x1 *= pi/180.0
   y2 *= pi/180.0
   x2 *= pi/180.0
   #
   # approximate great circle distance with law of cosines
   #
   if(x1 == x2 and y1 == y2 ):
       return 0
   else:
       return acos( sin(y1)*sin(y2) + cos(y1)*cos(y2)*cos(x2-x1) ) * R
   #
#

def convertToCity(letter):
    return cityNames[letter];

def convertToLetter(city):
    for letter in cityNames.keys():
        if(cityNames[letter] == city):
            return letter


def makefile():
    file = open('romNodes.txt' , 'r')
    infile = file.read().split()
    x = 0;
    edges = {}
    while( x < len(infile) ):
        name = infile[x];
        lon = infile[x+1];
        lat = infile[x+2];
        coordinates = [lon, lat]
        edges[name] = coordinates
        x+=3
    #print(edges)
    file = open('romEdges.txt' , 'r')
    infile = file.read().split()
    neighbors = {}

    x = 0;
    while( x < len(infile)):
        if infile[x] not in neighbors.keys():
            temp = [(infile[x+1])]
            neighbors[infile[x]] = temp
        else:
            temp = neighbors[infile[x]];
            temp.append(infile[x+1])
            neighbors[infile[x]] = temp

        if infile[x+1] not in neighbors.keys():
            temp = [(infile[x])]
            neighbors[infile[x+1]] = temp
        else:
            temp = neighbors[infile[x+1]];
            temp.append(infile[x])
            neighbors[infile[x+1]] = temp
        x+=2
    #print (neighbors);

    file = open('romFullNames.txt' , 'r')
    infile = file.read().split()
    cities = {}
    x = 0
    while( x < len(infile) ):
        letter = (infile[x])[0]
        city = infile[x]
        cities[letter] = city
        x+=1;
    #print (cities);
    	
    maxout = open ( 'edgesR.pkl', 'wb')
    dump(  edges , maxout , protocol = 2 )
    maxout.close()

    fout = open( 'neighborsR.pkl' , 'wb' )
    dump( neighbors , fout , protocol = 2 )
    fout.close()

    lastout = open( 'citiesR.pkl' , 'wb' )
    dump( cities , lastout , protocol = 2 )
    lastout.close()
def buildPath(parent, start, end):
    path = []
    path.append(end)
    current = end
    totald = 0
    while(current != start):
        p, dist = parent[current]
        totald += dist
        path.append(p)
        current = p
    print 'total distance: ', totald
    return path 

##################
def printPath(path):
    path.reverse()
    end  = path.pop()
    for word in path:
        print(convertToCity(word) + ' ->'),
    print convertToCity(end)
    print "It takes " + str(len(path)) + " steps to get from " + convertToCity(path[0]) + " to " + convertToCity(end)

#---------------------------------------------------------------------
# Main
print

makefile();
locations = load(open('edgesR.pkl','rb'))
NeighborsDictionary = load(open('neighborsR.pkl','rb'))
cityNames = load(open('citiesR.pkl','rb'))

start = convertToLetter(start);
target = convertToLetter(target);

print 'A* Search'
print '-----------------------------'
parent = findPathAStar(NeighborsDictionary, locations, start, target)
path = buildPath(parent, start, target)
printPath(path);

print
print 'Uniform Cost Search'
print '-----------------------------'
parent = findPathUniformCost(NeighborsDictionary, locations, start, target)
path = buildPath(parent, start, target)
printPath(path);

print






