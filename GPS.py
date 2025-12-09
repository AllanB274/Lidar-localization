#Prend la liste des coordonnées des balises dont l'ordre a déjà été trouvé, ça fait la moyenne des 3 coordonnées calculées avec chacune des balises (j'ai mis dans le référentiel où le 0,0 est dans le coin de la table)
def GPS(lcobal):
    mx = (-lcobal[0][0] + 3.-lcobal[1][0] - lcobal[2][0])/3
    my = (-lcobal[0][1] + 1.-lcobal[1][1] + 2.-lcobal[2][1])/3
    return (mx, my)








