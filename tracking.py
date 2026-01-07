from GPS import voisins
import numpy as np


def filtre_point(L,r,a,a_balise,d_balise):
    Liste_point=[]
    for point in L:
        #on doit verifier que les points sont biens dans le cone de tolerance et de bonne qualité
        if (point.angle-(a_balise-a/2))*(point.angle-(a_balise+a/2))<0:
            if (point.dist-(d_balise-r/2))*(point.dist-(d_balise+r/2))<0:
                if point.qualite>200:
                    Liste_point.append(point)
    return Liste_point
    


def tracking(L,pos,a_balise,r,a):
    #voir schema pour comprendre les appélation d'angle
    
    #L : liste de point
    #pos : ancienne position du robot
    #d_balise : ancienne distance de la balise
    #d_balise_new : distance de la balise retrouvée
    #a : ouverture du cone
    #r : epaisseur du cone
    #a_balise : angle de la balise par rapport au référentiel robot
    #a_balise_new : angle de la balise retrouvée
    #beta : angle entre x table et robot
    #xb1,yb1 : coordonné de la balise avant dans le repere table avec origine robot
    #xb2,yb2 : idem avec la balise retrouvée
    #alpha :angle pour projeter la balise dans la base table
    
    x,y=pos[0],pos[1]
    d_balise=np.sqrt(x**2+y**2)
    
    points_filtrés=filtre_point(L,r,a,a_balise,d_balise) #on ne garde que les points qui sont dans dans le cone de recherche
    paquets=voisins(100,points_filtrés) #on forme des paquets (normalement un seul paquets devrait etre former
    paquets_tries=sorted(paquets,key=lambda x : x.nb, reverse=True) #tri les paquets en fonction de leur taille par ordre decroissant
    balise=paquets_tries[0] #la balise sera le paquets le plus gros
    
    a_balise_new=balise.angle
    d_balise_new=balise.dist
    
    beta=np.arctan(y/x)
    alpha=a_balise-a_balise_new+beta-np.pi 
    
    xb2=d_balise_new*np.cos(alpha)
    yb2=d_balise_new*np.sin(alpha)
    
    xb1=d_balise*np.cos(beta-180)
    yb1=d_balise*np.sin(beta-180)
    
    delta_x, delta_y = xb1-xb2, yb1-yb2 #deplacement du robot
    new_x, new_y = x+delta_x, y+delta_y #calcul nouvelle position
    
    return ((new_x,new_y),a_balise_new) # la position du robot et le nouvelle angle de la balise


