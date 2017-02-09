'''
Created on 30 déc. 2016

@author: remi
'''
#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from scipy import misc
import time
import heapq

class Noeud(object):
    def __lt__(self, other):
        return comparer2Points(self, other)    
    
    def __init__(self,posX,posY,cout=0,heuristique=0,parent=False,obstacle=0):
        self.cout=cout
        self.heuristique=heuristique
        self.posX=posX
        self.posY=posY
        self.parent=parent
        self.obstacle=obstacle
            
    def _get_posX(self):
        return(self._posX)
    def _set_posX(self,posX):
        self._posX=posX
        
    posX=property(_get_posX,_set_posX)
    def _get_posY(self):
        return(self._posY)
    def _set_posY(self,posY):
        self._posY=posY
        
    posY=property(_get_posY,_set_posY)

    def _get_heuristique(self):
        return(self._heuristique)
    def _set_heuristique(self,heuristique):
        self._heuristique=heuristique
    heuristique=property(_get_heuristique,_set_heuristique)
    def _get_parent(self):
        return(self._parent) 
    def _set_parent(self,parent):
        self._parent=parent
    parent=property(_get_parent,_set_parent)
    def __eq__(self,other):
        return ((self._get_posX()==other._get_posX()) and (self._get_posY()==other._get_posY()))
    
    def __lt__(self,other):
        return (self.heuristique<other.heuristique)
    def __gt__(self,other):
        return (self.heuristique>other.heuristique)

class A_star(object):
    def __init__(self, noeudI,noeudF,wold_map,heuristique):
        self.noeudI=noeudI
        self.noeudF=noeudF
        self.wold_map=wold_map
        self.heuristique=heuristique # a fonction wich given a node give
    def start(self):
        q_liste=[]
        q=heapq.heappush(q_liste, self.noeudI) # q is the heap wich give us
        


def NoeudDWN(noeud,Map):
    return(Map[noeud.posY+1][noeud.posX])      
def NoeudUP(noeud,Map):
    return(Map[noeud.posY-1][noeud.posX])
        
def NoeudWST(noeud,Map):
    return(Map[noeud.posY][noeud.posX+1])
        
def NoeudEST(noeud,Map):
    return(Map[noeud.posY][noeud.posX-1])
    
def distance(noeud1,noeud2):
    return(abs(noeud1.posX-noeud2.posX)+abs(noeud1.posY-noeud2.posY))         
    
def reconstituerChemin(noeud):
    l=[]        
    while noeud!=False:
        l.insert(0,noeud)
        noeud=noeud.parent
    return(l)
        
def position(noeud):
    return(noeud.posX+'   '+noeud.posY)
        
        
def planificationRoute(noeudI, noeudF, mapSeen, Map, n, m):
    openList= []
    heapq.heappush(openList,(0,noeudI))
    i=0
    while (openList!=[]):
        l=heapq.heappop(openList)
        noeud=l[1]
        cout=noeud.cout
        if(noeud.posX==noeudF.posX and noeud.posY==noeudF.posY):
            l=reconstituerChemin(noeud)
            return(l)
                
        """on cree les noeuds voisins
        """
        if(m-1>noeud.posX>0):
            if(n-1>noeud.posY>0):
                Voisin=[NoeudUP(noeud,mapSeen),NoeudDWN(noeud,mapSeen),NoeudWST(noeud,mapSeen),NoeudEST(noeud,mapSeen)]
            if(n-1==noeud.posY):
                Voisin=[NoeudUP(noeud,mapSeen),NoeudWST(noeud,mapSeen),NoeudEST(noeud,mapSeen)]
            if(0==noeud.posY):
                Voisin=[NoeudDWN(noeud,mapSeen),NoeudWST(noeud,mapSeen),NoeudEST(noeud,mapSeen)]
                
        if(m-1==noeud.posX):
                
            if(n-1>noeud.posY>0):
                Voisin=[NoeudUP(noeud,mapSeen),NoeudDWN(noeud,mapSeen),NoeudEST(noeud,mapSeen)]
            if(n-1==noeud.posY):
                Voisin=[NoeudUP(noeud,mapSeen),NoeudEST(noeud,mapSeen)]
            if(0==noeud.posY):
                Voisin=[NoeudDWN(noeud,mapSeen),NoeudEST(noeud,mapSeen)]
        if(0==noeud.posX):
            if(n-1>noeud.posY>0):
                Voisin=[NoeudUP(noeud,mapSeen),NoeudDWN(noeud,mapSeen),NoeudWST(noeud,mapSeen)]
            if(n-1==noeud.posY):
                Voisin=[NoeudUP(noeud,mapSeen),NoeudWST(noeud,mapSeen)]
            if(0==noeud.posY):
                Voisin=[NoeudDWN(noeud,mapSeen),NoeudWST(noeud,mapSeen)]
            
        for v in Voisin:
                
            if((v.cout>noeud.cout+1 or v.cout<=-1) and v.obstacle==0):
                v.cout=cout+1
                v.heuristique=cout+1+distance(v,noeudF)
                v.parent=noeud
                mapSeen[v.posY][v.posX]=v
                heapq.heappush(openList,(v.heuristique,v))
        i=i+1
    return('impossible de trouver un chemin')
    
def cheminValide(planifroute,mapSeen,caseSeen):
    boolean=True
    i=0
    n=len(planifroute)
    while(boolean and i<n):
        boolean1=caseSeen[planifroute[i].posY][planifroute[i].posX]
        boolean2=(mapSeen[planifroute[i].posY][planifroute[i].posX].obstacle==0)
        boolean=(boolean1 and boolean2)
        i=i+1
    return(boolean)
      
def comparer2Points(noeud1,noeud2):
    if(noeud1.heuristique==-1):
        return False
    elif(noeud2.heuristique==-1):
        return True
    elif(noeud1.heuristique<noeud2.heuristique):
        return True
    elif(noeud1.heuristique==noeud2.heuristique):
        return False
    else:
        return False
    
def binarise(tab, seuil):
    '''
    Cette fonction renvoie un tableau repr�sentant l'image binaris�e
    '''
    hauteur=len(tab)
    largeur=len(tab[0])
    for i in range (0,hauteur,1) :
        for j in range (0,largeur,1) :
            #print(i,j)
            if(tab[i][j] >= seuil):
                tab[i][j] = 0
            else:
                tab[i][j] = 1
    return (tab,hauteur,largeur)
        
def initMap(n,m):
    mapSeen=[]
    caseSeen=[]
    for i in range(n):
        mapSeen.append([0]*m)
        caseSeen.append([0]*m)
        for j in range (m):
            mapSeen[i][j]=Noeud(j,i,cout=-1)
            caseSeen[i][j]=False
    return mapSeen,caseSeen

def see(noeud, mapSeen, caseSeen, Map, champVision,n,m):
    posX=noeud.posX
    posY=noeud.posY
    for i in range (posY - champVision, posY + champVision):
        if(0<=i and i<n):
            if(i<=posY):
                for j in range(posX+(posY-champVision-i),posX-(posY-champVision-i)):
                    if(0<=j<m):
                        if(caseSeen[i][j]==False):
                            v=mapSeen[i][j]
                            v.obstacle=Map[i][j]
                            mapSeen[i][j]=v
                            caseSeen[i][j]=True
            if(i>posY):
                for j in range(posX-(posY+champVision-i),posX+(posY+champVision-i)):
                    if(0<=j<m):
                        if(caseSeen[i][j]==False):
                            v=mapSeen[i][j]
                            v.obstacle=Map[i][j]
                            mapSeen[i][j]=v
                            caseSeen[i][j]=True
                        
    
def Trajet(posX_Init,posY_Init,posX_F,posY_F,Map, champVision,n,m):
    noeudI=Noeud(posX_Init,posY_Init)
    noeudF=Noeud(posX_F,posY_F)
    noeudI.heuristique=distance(noeudI,noeudF)
    mapSeen,caseSeen=initMap(len(Map),len(Map[0]))
    mapSeen[posY_Init][posX_Init]=noeudI
    caseSeen[posY_Init][posX_Init]=True
    cheminValid=False
    possible=True
    i=0
    while(cheminValid==False and possible==True):
        liste=planificationRoute(noeudI,noeudF,mapSeen,Map,n,m)
        if(liste=='impossible de trouver un chemin'):
            return(liste)
        cheminValid=cheminValide(liste,mapSeen,caseSeen)
        compteAZero(mapSeen,n,m)
        for v in liste:
            see(v,mapSeen, caseSeen,Map,champVision,n,m)
        mapSeen[posY_Init][posX_Init].cout=0
        print(i)
        i=i+1
    if(possible):            
        return(liste)
    else:
        print('impossible de trouver un chemin')
        return('impossible de trouver un chemin')
            
def compteAZero(mapSeen,n,m):
    for i in range(n):
        for j in range(m):
            v=mapSeen[i][j]
            v.cout=-1
            v.heuristique=-1
            v.parent=False
            mapSeen[i][j]=v                
def copy(Map):
        
    n=len(Map)
    m=len(Map[0])
    l=np.ones((n,m),int)
    for i in range(n):
        for j in range(m):
            l[i][j]=Map[i][j]
    return l
        
def visualisation(Map,liste):
    l=copy(Map)
    if(liste=='impossible de trouver un chemin'):
        return l
    else:
        for v in liste:
            j=v.posY
            i=v.posX
            l[j][i]=2
        return l
    
def imagerize(chemin,n,m):
    for i in range(n):
        for j in range(m):
            if(chemin[i][j]==0):
                chemin[i][j]=255
            elif(chemin[i][j]==1):
                chemin[i][j]=0
            elif(chemin[i][j]==2):
                chemin[i][j]=125
    return(chemin)


image_init=misc.imread('Carte2.png')
t0=time.clock()
tab,n,m=binarise(image_init,125)
print("algo start")
l=Trajet(0,0,n-1,m-1,tab,champVision,n,m)
print(time.clock()-t0)

plt.imshow(imagerize(visualisation(tab,l),n,m),aspect='auto')
plt.show()