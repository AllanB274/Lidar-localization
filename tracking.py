from GPS import voisins, Point, f_least_square
import numpy as np
from scipy.optimize import least_squares


def filtre_point(L,r,a,a_balise,d_balise):
    # L : liste de tous les points
    # r :epaisseur de l'arc de recherche
    # a : angle d'ouverture de l'arc de recherche
    #a_balise : angle de la balise par rapport au robot
    #d_balise : distance de la balise au robot
    
    Liste_point=[]
    for point in L:
        #on doit verifier que les points sont biens dans le cone de tolerance et de bonne qualité
        if (point.angle-(a_balise-a))*(point.angle-(a_balise+a))<0:
            if (point.dist-(d_balise-r))*(point.dist-(d_balise+r))<0:
                if point.qualite>200:
                    Liste_point.append(point)
    return Liste_point
    
def retrouver_balise(L,balise,a,r):
    #L : liste de tous les points
    #balise : balise à retrouver de type point
    #a : angle d'ouverture de l'arc de recherche
    #r : epaisseur de l'arc de recherche
    
    points_filtrés=filtre_point(L,r,a,balise.angle,balise.dist) #on ne garde que les points qui sont dans dans le cone de recherche
    if len(points_filtrés)==0:
        return None
    paquets=voisins(100,points_filtrés) #on forme des paquets (normalement un seul paquets devrait etre former
    paquets_tries=sorted(paquets,key=lambda x : x.nb, reverse=True) #tri les paquets en fonction de leur taille par ordre decroissant
    new_balise=paquets_tries[0] #la balise sera le paquets le plus gros

    return new_balise


def tracking(L,X0,balises,a,r):
    #L: liste de points
    #coords_balises : liste des anciennes balises dans la base robot de type Point
    #a : angle d'ouverture de l'arc de recherche
    #r : rayon d'epaisseur de l'arc de recherche
    ptt={0:(3094,1950),1:(-94,1000),2:(3094,5),3:(1705,2094)} #balises theoriques
    
    def balise_perdue(X1,X0):
        x,y,theta=X0[0],X0[1],X0[2]
        xb,yb=X1[0],X1[1]
        a=np.pi-theta+np.atan2(y-yb,x-xb)
        d=np.sqrt((x-xb)**2+(y-yb)**2)
        return Point(angle=a,distance=d)
    
    nouvelles_balises=[]
    for b in balises:
        if b!=None:
            nouvelles_balises.append(retrouver_balise(L,b,a,r).centre) #on retrouve les balises une par une
        else:
            nouvelles_balises.append(retrouver_balise(L,balise_perdue(ptt[balises.index(b)],X0),a,r).centre)
    
    liste_pour_ls = []
    for i,j in enumerate(nouvelles_balises):
        if j != None:
            liste_pour_ls.append(((j.x, j.y), ptt[i]))
    coos = least_squares(f_least_square, X0, args=(liste_pour_ls))
    return {"robot":coos,"balises":nouvelles_balises}

    
    
     



    
    
    
    



