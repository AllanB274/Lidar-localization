from get_amalgames import Paquet, Point,distance
import numpy as np

def GPS(L):   #liste de balises
    (b1,b2,b3)=(L[0].centre,L[1].centre,L[2].centre)
    theta_1=abs(b2.angle-b3.angle)
    theta_2=abs(b1.angle-b3.angle)
    theta_3=abs(b1.angle-b2.angle)
    b=b1.dist
    c=b2.dist
    a=distance(b1,b2)
    alpha=np.arccos((b-c*np.cos(theta_3))/a)
    return(b*np.cos(alpha+0.322),abs(b*np.sin(alpha+0.322)))



            


