import numpy as np

def desiredHeadingLOS(x, y, prevwpx, prevwpy, nextwpx, nextwpy, radius):
    '''
    The LOS Heading finder as given in the tutorial code
    '''
    alpha = np.arctan2(nextwpy-prevwpy,nextwpx-prevwpx)
    los_s = ((x-prevwpx)*np.cos(alpha))+((y-prevwpy)*np.sin(alpha))
    los_e = -((x-prevwpx)*np.sin(alpha))+((y-prevwpy)*np.cos(alpha))
    los_delta = 0.0
    if radius > abs(los_e):
        los_delta = (radius**2 - los_e**2)**0.5
    xproj = los_s*np.cos(alpha) + prevwpx
    yproj = los_s*np.sin(alpha) + prevwpy 
    losx = xproj + los_delta*np.cos(alpha)-x
    losy = yproj + los_delta*np.sin(alpha)-y 
    los_heading = np.arctan2(losy,losx)
    if los_heading < -np.pi:
        los_heading += np.pi
    elif los_heading > np.pi:
        los_heading -= np.pi
    return los_heading

def reached(x,y, wpx, wpy, acceptance_radius):
    '''
    Checker to see if a position is within the acceptance radius of a waypoint
    '''
    d = False
    distance2waypoint = np.sqrt((x - wpx)**2 + (y - wpy)**2)
    if distance2waypoint < acceptance_radius:
         d = True
    return d