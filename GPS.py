#Prend la liste des coordonnées des balises dont l'ordre a déjà été trouvé, ça fait la moyenne des 3 coordonnées calculées avec chacune des balises (j'ai mis dans le référentiel où le 0,0 est dans le coin de la table)
def coordonnees_balises(L):
    def projeter(l,theta):
        for b in l:
            b.centre.x=b.centre.distance*np.cos(theta)
            b.centre.y=b.centre.distance*np.sin(theta)
        return l
    theta=2*np.pi-L[1].centre.angle
    return projeter(L,theta)
    


def GPS(balises_proj):
    mx = (-lcobal[0][0] + 3.-lcobal[1][0] - lcobal[2][0])/3
    my = (-lcobal[0][1] + 1.-lcobal[1][1] + 2.-lcobal[2][1])/3
    return (mx, my)










