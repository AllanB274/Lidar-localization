import numpy as np
from scipy.optimize import least_squares


class Point:
    def __init__(self,angle=None,distance=None,qualite=None,x=None,y=None):
        if angle!=None and distance!=None: #si vous voulez definir le point par ses coordonnées polaires
            self.angle=angle
            self.dist=distance
            self.x=distance*np.cos(angle)
            self.y=distance*np.sin(angle)
        elif x!=None and y!=None: #si vous voulez definir le point par ses coordonnées cartésiennes
            self.x=x
            self.y=y
            self.angle=np.arctan(y/x)
            self.dist=np.sqrt(x**2+y**2)
        else:
            raise("Pb get_amalgames.py Point()")
        self.qualite=qualite
        
class Paquet: #ensemble de points definis par un centre et un diamètre (le diamètre size ne sert que pour l'affichage des paquets)
    def __init__(self,L):
        self.nb=len(L)
        self.centre=Point(angle=np.mean([p.angle for p in L]),distance=np.mean([p.dist for p in L]),qualite=255) #tu définis le centre en polaire, c'est parfait
        self.size=max([distance2(p,self.centre) for p in L]) if len(L)>0 else 0
        ### self.size=max([distance2(p,self.centre) for p in L]) if len(L)>0 else 0

def f_least_square(coordexp): #le problème c'est que la fonction ne doit avoir qu'un seul argument, donc soit on y met les coordonnées de la table, soit il faut calculer les coordonnées théoriques dans la fonction en ne prenant qu'un seul argument ou alors en mettant des variables globales mais je suis pas sûr que ça soit vraiment possible
    lcoordstheo = [(0,0),(3,1),(0,2)]
    return [(coordsexp[0]-i[0],coordsexp[1]-i[1]) for i in lcoordstheo]

def distance(p1,p2): #calcule la distance entre deux points à partir de leurs coordonnées cartésiennes
    return np.sqrt((p2.x-p1.x)**2+(p2.y-p1.y)**2)

def distance2(p1,p2):
    return np.sqrt(p1.dist**2+p2.dist**2-2*p1.dist*p2.dist*np.cos(p1.angle-p2.angle))

def filtre_points(points): #filtre les points en fonction de leur qualité et de leur distance
    points_propres=[]
    for p in points:
        if p.dist < np.sqrt(2000**2+3000**2) and p.dist > 10 and p.qualite > 200: #si c'est pas trop loin de la table et que la qualité est supérieure à 200/255
            points_propres.append(p)
    return points_propres

def filtre_paquets(paquets,res): #filtre les paquets en fonction du nombre de points qui les composent
    paquets_filtres=[]
    alpha=res*(np.pi/180)
    for p in paquets:
        nb_max=1+(100//(2*p.centre.dist*np.sin(alpha/2))) #plus le paquet est loin, moins il contient de points. Si il y a trop de points alors ce n'est pas une balise
        if p.nb<=nb_max:
            paquets_filtres.append(p)
    return paquets_filtres

res=0.7 #résolution angulaire. Elle est normalement de 0.788 mais on en est pas sûr. A remplacer si plus d'informations sur la précision angulaire

def voisins(eps,points):
    paquets=[]
    traite=[]
    a_traiter=[]
    for p in points:    #on prend un premier point
        if p not in traite: #si ce point n'est pas deja traité
            a_traiter.append(p) #on l'ajoute dans une liste a_traiter
            g=[]                #on crée un nouveau groupe g
            while a_traiter !=[]: #tant qu'il reste des points à traiter :
                i=a_traiter.pop()    #on traite le premier point de la liste de points à traiter en le retirant de celle-ci
                traite.append(i)     #on ajoute ce point dans la liste traitée
                g.append(i)            #on l'ajoute dans ce nouveau groupe g
                for j in points:        #on parcourt les autres points
                    if j not in traite:       #s'ils ne sont pas déjà traités
                        if j not in a_traiter:    #s'ils ne sont pas déjà dans à traiter
                            ### if distance2(i,j)< eps :
                            if distance2(i,j)< eps :   #si le point qu'on est en train de traiter et celui-là sont suffisament proches
                                a_traiter.append(j)    #on ajoute ce deuxième point dans a_traiter, sinon on passe aux points suivants
            paquets.append(Paquet(g))                    #notre groupe est maintenant un paquet qu'on ajoute à la liste des paquets
    return paquets

def robot_in_balises(balises, return_rotated=False):

    #pour verifier que les balises encadrent bien le robot, on crée un rectangle formé par les quatres balises, on le tourne et on verifie que le robot est dans le rectangle
    def R(theta):
        # Matrice de rotation dans le sens inverse (peu importe)
        return np.matrix([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
    (s,b1,b2)=(balises[1],balises[0],balises[2]) #sommet, balise en haut, balise en bas
    m=Point(qualite=255,x=np.mean([b1.centre.x,b2.centre.x]),y=np.mean([b1.centre.y,b2.centre.y]))    #milieu du segment (b1,b2)
    delta_x=abs(m.x-s.centre.x) 
    delta_y=abs(m.y-s.centre.y)
    theta=np.arctan(delta_y/delta_x) #on calcul l'angle necessaire a la rotation du rectangle
    rotated = R(theta)@np.matrix([[b.centre.x for b in balises], [b.centre.y for b in balises]]) #on rotationne
    rotated = np.array(rotated)

    minx, maxx = min(rotated[0]), max(rotated[0]) #on forme le rectangle en prenant le max et min du triangle à gauche et à droite
    miny, maxy = min(rotated[1]), max(rotated[1])
    return minx*maxx<0 and miny*maxy<0


def trouver_balises(paquets, eps=250):
    #ici on veut trouver si 4 paquets sont des balises candidates
    def bonne_distance(j,balises,X,eps): #balise candidate, balises validées, distances souhaitées à ces balises et précision
        #on prend une balise candidate et on vérifie qu'elle est aux distances souhaitées des autres balises
        for b in balises:
            ### d=distance2(j.centre,b.centre)
            d=distance2(j.centre,b.centre)
            for e in X:
                if abs(d-e)<eps:       #si la distance entre la balise candidate et la balise b déjà choisie appartient à notre liste de distances alors c'est ok et on retire la distance de la liste
                    X.remove(e)
        return len(X)==0
        
    d1=np.sqrt(3000**2+1000**2) #distance longue entre balises
    d2=2000 #distance courte entre balises
    for p in paquets:     #on prend un premier point 
        for i in paquets: #on prend un deuxième point
            ### if abs(distance2(p.centre,i.centre)-d1)<eps:
            if abs(distance2(p.centre,i.centre)-d1)<eps: #si ces deux points sont à distance du grand côté du triangle :
                for j in paquets:                       # on cherche un troisième point 
                    if bonne_distance(j,[p,i],[d2,d1],eps):    # si ce-dernier est à bonne distance des deux autres:
                        for k in paquets:                      #on cherche la dernière balise
                            if bonne_distance(k,[p,i,j],[1300,np.sqrt(1000**2+1700**2),np.sqrt(2000**2+1300**2)],eps):  #si y'en a une à bonne distance des trois autres
                                ### triangle=sorted([p,i,j], key=lambda x: distance2(x.centre,k.centre))
                                triangle=sorted([p,i,j], key=lambda x: distance2(x.centre,k.centre))        #on les trie pour les identifier. comme ça on peut retrouver laquelle est laquelle rapidement
                                if robot_in_balises(balises=triangle):        #si les balises contournent une table dans laquelle se trouve le robot alors c'est possiblement nos balises !
                                    return triangle+[k]                      #on renvoie une liste de 4 paquets qui sont nos balises
    return None


def trouver_position(L):   #liste de balises

    #je vais mettre x1,y1,x2,y2,x3,y3 en parametre

    (b1,b2,b3)=(L[0].centre,L[1].centre,L[2].centre)
    theta=abs(b1.angle-b3.angle)
    b=b1.dist
    c=b3.dist
    a=distance2(b1,b3)
    alpha=(np.pi/2)-np.arccos((b-c*np.cos(theta))/a)
    return(b*np.cos(alpha),abs(b*np.sin(alpha)))

def trouver_position2(L):
    (b1,b2,b3)=(L[0].centre,L[1].centre,L[2].centre)
    
    #les coordonnées des balises 2 et 3 (1 etant l'origine à (0,0))
    y3=distance2(b1,b3)
    y2=y3/2
    x2=np.sqrt(distance2(b1,b2)**2-y2**2)
    
    #distance pour trilatération
    d1=b1.dist
    d2=b2.dist
    d3=b3.dist 
    y=(d1**2-d3**2+y3**2)/(2*y3)
    x=(d1**2-d2**2+x2**2+y2**2-(2*y2*y))/(2*x2)
    
    return (x,y)
    
def bilateration(L,coord1,coord2,coord3):
    #L = liste de balises
    # ATTENTION x1, y1 doivent être les coordonnées de la balise d'origine (0,0)
    # coord 1,2 et 3 sont les coordonnées théoriques des trois premières balises dans L
    #pour l'instant x1 et y1 valent (0,0) mais je travaille sur une version plus générale
    b1,b2,b3=L[0].centre,L[1].centre,L[2].centre
    d1=b1.dist
    d2=b2.dist
    d3=b3.dist
    (x1,y1)=coord1
    (x2,y2)=coord2
    (x3,y3)=coord3
    
    # on doit trouver les deux positions possibles du robot
    #calcul à vérifier
    D=np.sqrt((x2-x1)**2+(y2-y1)**2) #distance theorique entre les deux balises
    A=np.sqrt(4*D**2*d1**2 - (d1**2 - d2**2 + D**2)**2) #terme commun aux 4 valeurs

    xa  = x1 + (d1**2 - d2**2 + D**2)*(x2 - x1)/(2*D**2) + (y2 - y1)*A/(2*D**2)
    ya  = y1 + (d1**2 - d2**2 + D**2)*(y2 - y1)/(2*D**2) - (x2 - x1)*A/(2*D**2)

    xb = x1 + (d1**2 - d2**2 + D**2)*(x2 - x1)/(2*D**2) - (y2 - y1)*A/(2*D**2)
    yb = y1 + (d1**2 - d2**2 + D**2)*(y2 - y1)/(2*D**2) + (x2 - x1)*A/(2*D**2)
    
    #on leve le doute sur la bonne position grâce à la troisieme balise
    da=np.sqrt((x3-xa)**2+(y3-ya)**2)
    db=np.sqrt((x3-xb)**2+(y3-yb)**2)   
    if abs(da-d3)<abs(db-d3):
        x,y=xa,ya
    else:
        x,y=xb,yb
    
    #fonction pour trouver les coordonnées des balises calculées dans le référentiel table
    def coord_balises_pratique(p,coord_robot,a1):
        b=p.centre
        (x,y)=coord_robot
        beta=np.arctan(y/x)     #angle table, robot
        alpha=b.angle+beta+np.pi-a1    #angle de rotation
        d=b.dist
        xb,yb=d*np.cos(alpha),d*np.sin(alpha)
        return (xb+x,yb+y)
    
    
    balises_pratiques=[coord_balises_pratique(i,(x,y),b1.angle) for i in L]
    coordsapprox = least_squares(f_least_square,(x,y))    #je suis pas sûr si je dois mettre les coordonnées dans le référentiel table ou pas
    return [coordsapprox,balises_pratiques]
    

def trilateration(L,x1,y1,x2,y2,x3,y3):
    (b1,b2,b3)=(L[0].centre,L[1].centre,L[2].centre)   #on extrait les trois balises 1,2 et 3
    d1=b1.dist   #distances des balises au robot
    d2=b2.dist
    d3=b3.dist 
    #trilateration:
    y=((x3-x1)*(d1**2-d2**2+x2**2-x1**2+y2**2-y1**2)-(x2-x1)*(d1**2-d3**2+x3**2-x1**2+y3**2-y1**2))/(2*((x3-x1)*(y2-y1)-(x2-x1)*(y3-y1)))
    x=(d1**2-d2**2+x2**2-x1**2+y2**2-y1**2-2*y*(y2-y1))/(2*(x2-x1))    
    return (x,y)
    

def GPS(L,res):
    points_propres=filtre_points(L)             #on filtre les points         
    paquets=voisins(100,points_propres)         #on créé les paquets
    paquets_filtres=filtre_paquets(paquets,res) #on filtre les paquets
    balises=trouver_balises(paquets_filtres)    #on trouve les balises en cherchant le triangle
    print(f"bil : {bilateration(balises,(0,0),(3000,1000),(0,2000))}")
    return (trilateration(balises[:-1],0,0,3000,1000,0,2000),balises)   #on trilateralise


