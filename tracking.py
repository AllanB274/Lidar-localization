from GPS import voisins
import numpy as np


def filtre_point(L,r,a,a_balise,d_balise):
    Liste_point=[]
    for point in L:
        if (point.angle-(a_balise-a/2))*(point.angle-(a_balise+a/2))<0:
            if (point.dist-(d_balise-r/2))*(point.dist-(d_balise+r/2))<0:
                Liste_point.append(point)
    return Liste_point
    


def tracking(L,pos,a_balise,r,a):
    #L : liste de point
    #pos : ancienne position du robot
    #d_balise : ancienne distance de la balise
    #d_balise_new : distance de la balise retrouvée
    #a : ouverture du cone
    #r : epaisseur du cone
    #a_balise : angle de la balise par rapport au référentiel robot
    #a_balise_new : angle de la balise retrouvée
    
    x,y=pos[0],pos[1]
    d_balise=np.sqrt(x**2+y**2)
    points_filtrés=filtre_point(L,r,a,a_balise,d_balise) #on ne garde que les points qui sont dans dans le cone de recherche
    paquets=voisins(100,points_filtrés) #on forme des paquets (normalement un seul paquets devrait etre former
    paquets_tries=sorted(paquets,key=lambda x : x.nb, reverse=True) #tri les paquets en fonction de leur taille par ordre decroissant
    balise=paquets_tries[0] #la balise sera le paquets le plus gros
    a_balise_new=balise.angle
    d_balise_new=balise.dist
    alpha=a_balise_new-a_balise #angle duquel on s'est déplacé
    delta_d=np.sqrt(d_balise_new**2+d_balise**2-2*(d_balise_new*d_balise*np.cos(alpha))) # al kashi pour trouver la distance dont on s'est deplacer
    phi=np.arccos(-(d_balise_new**2+delta_d**2+d_balise**2)/(2*delta_d*d_balise)) # al kashi pour trouver l'angle phi
    beta=np.arctan(y/x) #angle etre l'ax des x, la balise origine et le robot
    theta=np.pi-phi-beta
    delta_x=delta_d*np.cos(theta) #variation de x de déplacement
    delta_y=delta_d*np.sin(theta) #variation de y de déplacement
    
    new_x=x+delta_x #nouvelle coordonnée x
    new_y=y+delta_y #nouvelle coordonnée y
    
    return ((new_x,new_y),a_balise_new) # la position du robot et le nouvelle ange de la balise


