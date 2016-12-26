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
start = "Los Angeles"
target = "Washington DC"
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
        qCount+=1;
        x, city = heappop(upcomingHeap)
        visited.append(city)
        if(city == target):
            break;
        for neighbor in NeighborsDictionary[city]:
            if(neighbor not in visited and ( neighbor not in upcoming or gCost[neighbor] > gCost[city] + calcDistance(neighbor,city, locations ) ) ):
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
    file = open('rrNodes.txt' , 'r')
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
    # print(edges)

    file = open('rrEdges.txt' , 'r')
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
    # print (neighbors);

    file = open('rrNodeCity.txt' , 'r')
    infile = file.read().splitlines()
    cities = {}
    for line in infile:
         key, city = seperate(line)
         cities[key] = city
    # print (cities);

    maxout = open ( 'edgesUS.pkl', 'wb')
    dump(  edges , maxout , protocol = 2 )
    maxout.close()

    fout = open( 'neighborsUS.pkl' , 'wb' )
    dump( neighbors , fout , protocol = 2 )
    fout.close()

    lastout = open( 'citiesUS.pkl' , 'wb' )
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
        print(word + ' ->'),
    print convertToCity(end)
    print "It takes " + str(len(path)) + " steps to get from " + convertToCity(path[0]) + " to " + convertToCity(end)
def seperate(string):
    i = string.index(' ')
    return string[:i], string[(i+1):]

#---------------------------------------------------------------------
# Main
print

# makefile();
locations = load(open('edgesUS.pkl','rb'))
NeighborsDictionary = load(open('neighborsUS.pkl','rb'))
cityNames = load(open('citiesUS.pkl','rb'))
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






