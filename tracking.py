from GPS import voisins, Point, f_least_square
import numpy as np
from scipy.optimize import least_squares


def filtre_point(L,r,a,a_balise,d_balise):
    # L : liste de tous les points
    # r : demi epaisseur de l'arc de recherche
    # a : demi angle d'ouverture de l'arc de recherche
    #a_balise : angle de la balise par rapport au robot
    #d_balise : distance de la balise au robot
    
    Liste_point=[]
    for point in L:
        #on doit verifier que les points sont biens dans l'arc de recherche et de bonne qualité
        if point.qualite>200:
            if (point.angle-(a_balise-a))*(point.angle-(a_balise+a))<0:
                if (point.dist-(d_balise-r))*(point.dist-(d_balise+r))<0: 
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
    paquets_tries=sorted(paquets,key=lambda x : x.nb) #tri les paquets en fonction de leur taille
    new_balise=paquets_tries[0] #la balise sera le paquets le plus petit (sinon humain)

    return new_balise


def tracking(L,X0,balises,a,r):
    #L: liste de points
    #balises : liste des anciennes balises dans la base robot de classe Point
    #a : angle d'ouverture de l'arc de recherche
    #r : rayon d'epaisseur de l'arc de recherche
    ptt={0:(3094,1950),1:(-94,1000),2:(3094,5),3:(1700,2094)} #balises theoriques
    
    def position_balise_robot(X1,X0):
        #X1 : coords de la balise perdue dans la base table de type tuple
        #X0 : coords du robot
        x,y,theta=X0
        xb,yb=X1
        a=(np.pi-theta+np.atan2(y-yb,x-xb))%(2*np.pi) #angle balise par rapport au robot
        d=np.sqrt((x-xb)**2+(y-yb)**2) #distance balise robot
        return Point(angle=a,distance=d)
    
    nouvelles_balises=[]
    balises_supposees=[]
    for b in balises:
        b_s=position_balise_robot(ptt[balises.index(b)],X0) #balise supposee
        balises_supposees.append(b_s)
        balise_retrouvee=retrouver_balise(L,b_s,a,r)
        if balise_retrouvee != None :
            nouvelles_balises.append(balise_retrouvee.centre)
        else:
            nouvelles_balises.append(None)
    
    liste_pour_ls = []
    for i,p in enumerate(nouvelles_balises):
        if p != None:
            liste_pour_ls.append(((p.x, p.y), ptt[i]))
    if not X0.all(None):
        X0=(0, 0, 0)
    ls = least_squares(f_least_square, X0, args=(liste_pour_ls, ))
    coos = ls.x
    return {"robot":coos,"balises":nouvelles_balises,"confiance":ls.cost,"supp":balises_supposees}

    
    
     



    
    
    
    
