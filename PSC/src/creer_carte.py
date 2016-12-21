# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import misc
import time

image_init=misc.imread('D:\Polytechnique\PSC\Carte\Carte2.png') #Convertir image en tableau

n,m=image_init.shape

champVision=10 #Champ de vision du drone considéré comme constant

import heapq 



class Noeud(object):
    
    def __lt__(self, other): #définition d'un ordre entre les noeuds
        return Noeud.comparer2Points(self, other)    
    
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
    
    def _get_cout(self):
        return(self._cout)
    
    def _set_cout(self,cout):
        self._cout=cout
        
    cout=property(_get_cout,_set_cout)
    
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
    #Déplacements Nord/Sud/Est/Ouest
    def NoeudDWN(noeud,Map):
        return(Map[noeud.posY+1][noeud.posX])
        
    def NoeudUP(noeud,Map):
        return(Map[noeud.posY-1][noeud.posX])
        
    def NoeudWST(noeud,Map):
        return(Map[noeud.posY][noeud.posX+1])
        
    def NoeudEST(noeud,Map):
        return(Map[noeud.posY][noeud.posX-1])
    #Distance Vol d'oiseau
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
        
    def planificationRoute(noeudI,noeudF,mapSeen,Map,n,m): 
        '''
        Algorithme similaire Dijkstra
        heuristique= cout pour venir + distance au point final
        liste de priorité déjà implémenté avec heapq + définition ordre des noeuds
        '''
        openList= []
        heapq.heappush(openList,(0,noeudI))
        i=0
        while (openList!=[]):
            l=heapq.heappop(openList)
            noeud=l[1]
            cout=noeud.cout
            if(noeud.posX==noeudF.posX and noeud.posY==noeudF.posY):
                l=Noeud.reconstituerChemin(noeud)
                return(l)
                
            """on cree les noeuds voisins
            """
            if(m-1>noeud.posX>0):
                if(n-1>noeud.posY>0):
                    Voisin=[Noeud.NoeudUP(noeud,mapSeen),Noeud.NoeudDWN(noeud,mapSeen),Noeud.NoeudWST(noeud,mapSeen),Noeud.NoeudEST(noeud,mapSeen)]
                if(n-1==noeud.posY):
                    Voisin=[Noeud.NoeudUP(noeud,mapSeen),Noeud.NoeudWST(noeud,mapSeen),Noeud.NoeudEST(noeud,mapSeen)]
                if(0==noeud.posY):
                    Voisin=[Noeud.NoeudDWN(noeud,mapSeen),Noeud.NoeudWST(noeud,mapSeen),Noeud.NoeudEST(noeud,mapSeen)]
                
            if(m-1==noeud.posX):
                
                if(n-1>noeud.posY>0):
                    Voisin=[Noeud.NoeudUP(noeud,mapSeen),Noeud.NoeudDWN(noeud,mapSeen),Noeud.NoeudEST(noeud,mapSeen)]
                if(n-1==noeud.posY):
                    Voisin=[Noeud.NoeudUP(noeud,mapSeen),Noeud.NoeudEST(noeud,mapSeen)]
                if(0==noeud.posY):
                    Voisin=[Noeud.NoeudDWN(noeud,mapSeen),Noeud.NoeudEST(noeud,mapSeen)]
            if(0==noeud.posX):
                if(n-1>noeud.posY>0):
                    Voisin=[Noeud.NoeudUP(noeud,mapSeen),Noeud.NoeudDWN(noeud,mapSeen),Noeud.NoeudWST(noeud,mapSeen)]
                if(n-1==noeud.posY):
                    Voisin=[Noeud.NoeudUP(noeud,mapSeen),Noeud.NoeudWST(noeud,mapSeen)]
                if(0==noeud.posY):
                    Voisin=[Noeud.NoeudDWN(noeud,mapSeen),Noeud.NoeudWST(noeud,mapSeen)]
            
            for v in Voisin:
                
                if((v.cout>noeud.cout+1 or v.cout<=-1) and v.obstacle==0):
                    '''
                    print('/////////////////////////////////////////////////////////////////////////////')
                    print(Noeud.imprime(noeud),Noeud.imprime(v))
                    '''
                    v.cout=cout+1
                    v.heuristique=cout+1+Noeud.distance(v,noeudF)
                    v.parent=noeud
                    mapSeen[v.posY][v.posX]=v
                    '''
                    print(Noeud.imprime(v),Noeud.imprime(v.parent))
                    '''
                    heapq.heappush(openList,(v.heuristique,v))
            i=i+1
        return('impossible de trouver un chemin')
    
    #On veut savoir si tout le chemin passe par des zones inconnues
    def cheminValide(planifroute,mapSeen,caseSeen):
        boolean=True
        i=0
        n=len(planifroute)
        while(boolean and i<n):
            boolean1=caseSeen[planifroute[i].posY][planifroute[i].posX]
            boolean2=(mapSeen[planifroute[i].posY][planifroute[i].posX].obstacle==0)
            boolean=boolean1 and boolean2
            i=i+1
        return(boolean)
    '''    
    def ajouteOrdre(openList,v):
        #On imagine openList deja triee en fonction de leure heuristique
        if(openList==[]):
            return([v])
        noeud=openList[0]
        indice=0
        while(Noeud.comparer2Points(noeud,v)==True and indice+1<len(openList)):
            indice=indice+1
            noeud=openList[indice]
        return(openList[:indice]+[v]+openList[indice:])
            
    '''
    #ordre noeud
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
    
    #Pour affiner la conversion image -> tableau 
    def binarise(tab, seuil):
        """Cette fonction renvoie un tableau représentant l'image binarisée"""
        #Fonction à compléter
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
       
    #On voit rien au début
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

    #Mise à jour de ce que l'on voit
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
        noeudI.heuristique=Noeud.distance(noeudI,noeudF)
        mapSeen,caseSeen=Noeud.initMap(len(Map),len(Map[0]))
        mapSeen[posY_Init][posX_Init]=noeudI
        caseSeen[posY_Init][posX_Init]=True
        cheminValide=False
        possible=True
        i=0
        while(cheminValide==False and possible==True):
            liste=Noeud.planificationRoute(noeudI,noeudF,mapSeen,Map,n,m) #trouve un chemin
            if(liste=='impossible de trouver un chemin'):
                return(liste)
            cheminValide=Noeud.cheminValide(liste,mapSeen,caseSeen) #vérifie que le chemin passe par des endroits où on est sur de pouvoir passer
            '''
            print(Noeud.mapSeen(mapSeen))
            print(Noeud.visualisation(Noeud.mapSeen(mapSeen),liste))
            '''
            Noeud.compteAZero(mapSeen,n,m) #on remet les couts/heuristiques/parents/obstacle a zero car la carte mapSeen change
            for v in liste:
                 Noeud.see(v,mapSeen, caseSeen,Map,champVision,n,m) #carte mapSeen mise à jour, ie le drone suit le chemin liste pompé pour voir ce qu'il y a aux endroits inconnus
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
    '''           
    def imprime(noeud):
        return('({},{})'.format(noeud.posX,noeud.posY))
        
    def imprimeList(liste):
        n=len(liste)
        m=[0]*n
        for i in range(n):
            m[i]=Noeud.imprime(liste[i])
        return m
        
    def imprimeDeep(tab):
        n=len(tab)
        m=len(tab[0])
        l=[]
        for i in range(n):
            l.append([0]*m)
            for j in range(m):
                l[i][j]=Noeud.imprime(tab[i][j])
        return l
    
    def mapSeen(mapSeen):
        n=len(mapSeen)
        m=len(mapSeen[0])
        l=np.ones((n,m),int)
        for i in range(n):
            for j in range(m):
                l[i][j]=mapSeen[i][j].obstacle
        return(l)
    '''
    def copy(Map):
        
        n=len(Map)
        m=len(Map[0])
        l=np.ones((n,m),int)
        for i in range(n):
            for j in range(m):
                l[i][j]=Map[i][j]
        return l
        
    def visualisation(Map,liste):
        l=Noeud.copy(Map)
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

t0=time.clock()
tab,n,m=Noeud.binarise(image_init,125)
l=Noeud.Trajet(0,0,n-1,m-1,tab,champVision,n,m)
print(time.clock()-t0)
plt.subplot(211)
plt.imshow(tab)
plt.subplot(221)
plt.imshow(Noeud.imagerize(Noeud.visualisation(tab,l),n,m))
plt.show()
