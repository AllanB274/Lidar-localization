

from get_amalgames import Paquet, Point
import numpy as np

def projeter_balises(L):
    def projeter(l,theta):
        coord=[]
        for b in l:
            x=b.centre.x*np.cos(theta)-b.centre.y*np.sin(theta)   #on rotationne
            y=b.centre.y*np.cos(theta)+b.centre.x*np.sin(theta)
            coord.append((x,y))
        return coord
    theta=-L[1].centre.angle        # l'angle entre la base table et la base robot
    return projeter(L,theta)
    


def GPS(C):
    mx = (-C[0][0] + 3000.-C[1][0] - C[2][0])/3
    my = (-C[0][1] + 1000.-C[1][1] + 2000.-C[2][1])/3
    return (mx, my)











