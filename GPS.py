import numpy as np

#Convertit les coordonnées cartésiennes en polaires
def carttopol(coords):
    return np.sqrt(coords[0]**2+coords[1]**2), np.atan(coords[1]/coords[0])

#Calcule la distance entre deux balises par rapport au robot
def distpol(r1,r2,theta):
    return np.sqrt(r1**2+r2**2-2*r1*r2*np.cos(theta))

#Détermine les coordonnées de la balise de référence dans le référentiel du robot
def baliseref(bal1,bal2,balxy1,balxy2):
    if abs(bal1[1]*bal2[1]) > 3*(np.pi**2)/4:
        if min(bal1[1],bal2[1]) == bal1[1]:
            return balxy1
        else:
            return balxy2
    else:
        if max(bal1[1],bal2[1]) == bal1[1]:
            return balxy1
        else:
            return balxy2

#Détermine le duo des balises de référence
def recon(lbalises):
    lpol = [carttopol(i) for i in lbalises]
    D01 = distpol(lpol[0][0],lpol[1][0],lpol[1][1]-lpol[0][1])
    D02 = distpol(lpol[0][0],lpol[2][0],lpol[2][1]-lpol[0][1])
    D12 = distpol(lpol[1][0],lpol[2][0],lpol[2][1]-lpol[1][1])
    m = min(D01,D02,D12)
    if m == D01:
        return baliseref(lpol[0],lpol[1],lbalises[0],lbalises[1])
    elif m == D02:
        return baliseref(lpol[0],lpol[2],lbalises[0],lbalises[2])
    else:
        return baliseref(lpol[1],lpol[2],lbalises[1],lbalises[2])


def GPS(lbalises, coorbaliseref=(-1.5,1.0)):
    """Retourne les coordonnées GPS du robot dans le référentiel de la table, (0,0) est le centre.
    
    Entrées :
        lbalises : liste de tuples, tuples qui sont les coordonnées (x,y) des balises. L'ordre des balises n'importe pas.
        
        coorbaliseref : tuple des coordonnées de la balise de référence, par défaut (-1.5,1.).
    
    Sortie :
        Tuple des coordonnées du robot dans le référentiel de la table.
    """
    xy = recon(lbalises)
    return (coorbaliseref[0]-xy[0],coorbaliseref[1]-xy[1])

#Prend la liste des coordonnées des balises dont l'ordre a déjà été trouvé, ça fait la moyenne des 3 coordonnées calculées avec chacune des balises (j'ai mis dans le référentiel où le 0,0 est dans le coin de la table)
def GPS2(lcobal):
    mx = (-lcobal[0][0] + 3.-lcobal[1][0] - lcobal[2][0])/3
    my = (-lcobal[0][1] + 1.-lcobal[1][1] + 2.-lcobal[2][1])/3
    return (mx, my)






