from GPS import voisins, trilateration


def filtre_point(L,r,a,a_balise,d_balise):
    Liste_point=[]
    for point in L:
        #on doit verifier que les points sont biens dans le cone de tolerance et de bonne qualité
        if (point.angle-(a_balise-a/2))*(point.angle-(a_balise+a/2))<0:
            if (point.dist-(d_balise-r/2))*(point.dist-(d_balise+r/2))<0:
                if point.qualite>200:
                    Liste_point.append(point)
    return Liste_point
    
def retrouver_balise(L,balise,a,r):
    points_filtrés=filtre_point(L,r,a,balise.angle,balise.dist) #on ne garde que les points qui sont dans dans le cone de recherche
    paquets=voisins(100,points_filtrés) #on forme des paquets (normalement un seul paquets devrait etre former
    paquets_tries=sorted(paquets,key=lambda x : x.nb, reverse=True) #tri les paquets en fonction de leur taille par ordre decroissant
    new_balise=paquets_tries[0] #la balise sera le paquets le plus gros
    if "je sais pas encore quoi" :
        return None

    return new_balise


def tracking(L,balises,a,r):
    
    #L: liste de points
    #balises : liste des anciennes balises,
    #a : angle d'ouverture du cone
    #r : rayon d'epaisseur du cone
    
    nouvelles_balises=[]
    coords=[(0,0),(3000,1000),(0,2000),(-500,1200)]  #coordonnées des 4 balises
    for b in balises:
        L.append(retrouver_balise(L,b,a,r)) #on retrouve les balises une par une
    
    for b in range(len(nouvelles_balises)): 
        if nouvelles_balises[b] == None:         #si une balise est cachée
            nouvelles_balises.pop(b)            # on la retire
            coords.pop(b)                       # on retire sa coordonnées
            x1,y1,x2,y2,x3,y3=coords[0][0],coords[0][1],coords[1][0],coords[1][1],coords[2][0],coords[2][1]
            new_pos=trilateration(nouvelles_balises,x1,y1,x2,y2,x3,y3)
        else :
            new_pos=trilateration(nouvelles_balises[:-1],0,0,3000,1000,0,2000) #on utilise les trois balises principales
    return new_pos,nouvelles_balises



    
    
    
    



