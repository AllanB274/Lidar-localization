import numpy as np

class Point:
    def __init__(self,angle=None,distance=None,qualite=None,x=None,y=None):
        if angle!=None and distance!=None:
            self.angle=angle
            self.dist=distance
            self.x=distance*np.cos(angle)
            self.y=distance*np.sin(angle)
        elif x!=None and y!=None:
            self.x=x
            self.y=y
            self.angle=np.arctan(y/x)
            self.dist=np.sqrt(x**2+y**2)
        else:
            raise("Pb get_amalgames.py Point()")
        self.qualite=qualite
        
    def __init__(self, x, y, qualite):
        self.x=x
        self.y=y
        self.qualite=qualite
        self.angle=np.arctan(y/x)
        self.dist=np.sqrt(x**2+y**2)
        
class Paquet:
    def __init__(self,L):
        self.nb=len(L)
        self.centre=Point(angle=np.mean([p.angle for p in L]),distance=np.mean([p.dist for p in L]),qualite=255)
        self.size=max([distance(p,self.centre) for p in L])

        
def distance(p1,p2):
    return np.sqrt(abs((p2.x)-(p1.x))**2+abs((p2.y)-(p1.y))**2)


def filtre_points(points):
    points_propres=[]
    for p in points:
        if p.dist < np.sqrt(2000**2+3000**2) and p.dist > 10 and p.qualite > 200:
            points_propres.append(p)
    return points_propres

def filtre_paquets(paquets,res):
    paquets_filtres=[]
    alpha=res*(np.pi/180)
    for p in paquets:
        nb_max=1+(100//(2*p.centre.dist*np.sin(alpha/2)))
        if p.nb<=nb_max:
            paquets_filtres.append(p)
    return paquets_filtres

res=0.7 #résolution angulaire. elle est normalement de 0.788 mais on en est pas sûr. à remplacer si plus d'information sur la précision angulaire

def voisins(eps,points):
    paquets=[]
    traite=[]
    a_traiter=[]
    for p in points:
        if p not in traite:
            a_traiter.append(p)
            g=[]
            while a_traiter !=[]:
                i=a_traiter.pop()
                traite.append(i)
                g.append(i)
                for j in points:
                    if j not in traite:
                        if j not in a_traiter:
                            if distance(i,j)< eps :
                                a_traiter.append(j)
            paquets.append(Paquet(g))
    return paquets

def robot_in_balises(balises, return_rotated=False):
    def R(theta):
        # Matrice de rotation dans le sens inverse (peu importe)
        return np.matrix([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
    for i in balises:
        for j in balises:
            if i!=j:
                if (distance(i.centre,j.centre)-2000)<350:
                    a,b=i,j #la base
    for k in balises:
        if k!=a and k!=b:
            c=k #le sommet du triangle
    delta_x=abs(np.mean([a.centre.x,b.centre.x])-c.centre.x)
    delta_y=abs(np.mean([a.centre.y,b.centre.y])-c.centre.y)
    theta=np.arctan(delta_y/delta_x)
    rotated = R(theta)@np.matrix([[b.centre.x for b in balises], [b.centre.y for b in balises]])
    rotated = np.array(rotated)
    if return_rotated:
        rotateds = []
        for i in range(len(rotated[0])):
            p = Paquet([])
            p.nb=1
            p.centre = Point(x=rotated[0][i], y=rotated[1][i], qualite=255)
            p.size = balises[i].size
            rotateds.append(p)
            
        return rotated
    minx, maxx = min(rotated[0]), max(rotated[0])
    miny, maxy = min(rotated[1]), max(rotated[1])
    return minx*maxx<0 and miny*maxy<0


def trouver_balises(paquets, eps=250):
    
    def bonne_distance(j,balises,X,eps): #balise candidate, balises validées, distances souhaitaient à ces balises et precision
        for b in balises:
            d=distance(j.centre,b.centre)
            for e in X:
                if abs(d-e)<eps:
                    X.remove(e)
        return len(X)==0
    
    for p in paquets:
        for i in paquets:
            if abs(distance(p.centre,i.centre)-np.sqrt(3000**2+1000**2))<eps:
                for j in paquets:
                    if bonne_distance(j,[p,i],[2000,np.sqrt(3000**2+1000**2)],eps):
                        for k in paquets:
                            if bonne_distance(k,[p,i,j],[1300,np.sqrt(1000**2+1700**2),np.sqrt(2000**2+1300**2)],eps):
                                if robot_in_balises((p,i,j)):
                                    return(p,i,j,k)
    return None





