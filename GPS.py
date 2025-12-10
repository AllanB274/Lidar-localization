#Prend la liste des coordonnées des balises dont l'ordre a déjà été trouvé, ça fait la moyenne des 3 coordonnées calculées avec chacune des balises (j'ai mis dans le référentiel où le 0,0 est dans le coin de la table)
from get_amalgame import Paquet, Point

def coordonnees_balises(L):
    def projeter(l,theta):
        for b in l:
            b.centre.x=b.centre.distance*np.cos(theta)
            b.centre.y=b.centre.distance*np.sin(theta)
        return l
    theta=2*np.pi-L[1].centre.angle
    return projeter(L,theta)
    


def GPS(balises_proj):
    (b1,b2,b3)=(balises_proj[0],balises_proj[1],balises_proj[2])
    mx = (-b1.centre.x + 3.-b2.centre.y - b3.centre.x)/3
    my = (-b1.centre.y + 1.-b2.centre.y + 2.-b3.centre.y)/3
    return (mx, my)












