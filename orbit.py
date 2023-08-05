import cv2
import imageio
import numpy as np
from numpy import array
from math import sin, cos, tan, pi, sqrt, ceil, log, degrees, radians, atan2, gcd
from time import time_ns, sleep
from random import randint

class body:
    def __init__(self,position,mass,velocity,color,fixed):
        self.position = position
        self.mass = mass
        self.velocity = velocity
        self.color = color
        self.places = []
        self.fixed = fixed
    pass

def randBody():
    global size
    x = randint(-int(size/4),int(size/4)) + center[0]
    y = randint(-int(size/4),int(size/4)) + center[1]
    pos = (x,y)
    distance = getDistance(pos,(0,0))
    mass = randint(1,10)
    speed = randint(50,90) / 4
    angle = degrees(randint(0,360))
    color = [100,100,100]
    return createBody(pos,mass,speed,angle,color)

def createBody(pos,mass,speed,angle,color,fixed=False):
    velocity = polarConversion(radians(angle),pos,speed)
    return body(pos,mass,velocity,color,fixed)

def getDistance(pos1,pos2):
    x1 = pos1[0]
    y1 = pos1[1]
    x2 = pos2[0]
    y2 = pos2[1]
    delta_x = abs(x1 - x2)
    delta_y = abs(y1 - y2)
    distance = sqrt(delta_x**2 + delta_y**2)
    return distance

def vectorSum(vectors,pos):
    x = sum([vector[0] - pos[0] for vector in vectors]) + pos[0]
    y = sum([vector[1] - pos[1] for vector in vectors]) + pos[1]
    return (x,y)

def vectorMultiplacation(origin,vector,scaler):
    angle = getAngle(origin,vector)
    distance = getDistance(origin, vector) * scaler
    newVector = polarConversion(angle,origin,distance)
    return newVector

def getAngle(pos1,pos2):
    delta_x = pos2[0] - pos1[0]
    delta_y = pos2[1] - pos1[1]
    angle = atan2(delta_y,delta_x)
    return angle

def polarConversion(angle,origin,scaler):
    return (cos(angle) * scaler + origin[0],sin(angle) * scaler + origin[1])

def printAngle(name,angle):
    print(name,degrees(angle))

def createVector(pos1,pos2,scaler):
    angle = getAngle(pos1,pos2)
    vector = polarConversion(angle,pos1,scaler)
    return vector

def calcAcceleration(body,forceVector):
    pos = body.position
    mass = body.mass
    magnitude = getDistance(pos,forceVector)
    acelerationScaler = magnitude/mass
    acelerationVector = createVector(pos,forceVector,acelerationScaler)
    return acelerationVector

def gravity(body1,body2):
    pos1 = body1.position
    pos2 = body2.position
    mass1 = body1.mass
    mass2 = body2.mass
    r = getDistance(pos1,pos2)
    force=(G*mass1*mass2)/r**2
    vector = createVector(pos1,pos2,force)
    return vector

def TotalGravity(body,bodies):
    return vectorSum([gravity(body,body2) for body2 in bodies if body2.position != body.position],body.position)

def getForces(body,bodies):
    pos = body.position
    gravity = TotalGravity(body,bodies)
    # posible other forces
    forceSum = vectorSum([gravity],pos)
    return forceSum

def massRadius(body):
    return int(sqrt(body.mass))

def CenterOfMass(bodies):
    if len(bodies) > 0:
        # whatever this stands for
        Xtop = sum([body.mass * body.position[0] for body in bodies])
        Ytop = sum([body.mass * body.position[1] for body in bodies])
        # total mass
        mT = sum([body.mass for body in bodies])
        COMX = Xtop / mT
        COMY = Ytop / mT
        return (COMX,COMY)
    else:
        return (0,0)

def combineBody(pair,bodies):
    [body1,body2] = pair
    if not (body1.fixed or body2.fixed):
        pos1 = body1.position
        pos2 = body2.position
        v1 = body1.velocity
        m1 = body1.mass
        v2 = body2.velocity
        m2 = body2.mass
        mT = m1 + m2
        COM = CenterOfMass([body1,body2])
        p1 = vectorMultiplacation(COM,v1,m1)
        p2 = vectorMultiplacation(COM,v2,m2)
        pT = vectorSum([p1,p2],(COM))
        vT = vectorMultiplacation(COM,pT,1/mT)
        body3 = body(COM,mT,vT,[50,30,20],False)
        bodies.append(body3)
        if body1.mass >= body2.mass:
            body3.places = body1.places
        else:
            body3.places = body2.places
        bodies.remove(body1)
        bodies.remove(body2)
    elif body1.fixed:
        bodies.remove(body2)
    else:
        bodies.remove(body1)

def oneWay(body1,body2,bodies):
    return bodies.index(body2) > bodies.index(body1)

def intersect(body1,body2):
    return getDistance(body1.position,body2.position) < massRadius(body1) + massRadius(body2)

def addPlace(body):
    maxPos = 2200
    pos = body.position
    places = body.places
    places.append(pos)
    if len(places) > maxPos:
        places.pop(0)

def resolveIntersections(bodies):
    for body1 in bodies:
        for body2 in bodies:
            if oneWay(body1,body2,bodies) and intersect(body1,body2):
                combineBody([body1,body2],bodies)
                return resolveIntersections(bodies)

def updateBodies(bodies):

    resolveIntersections(bodies)

    # escapes = [body for body in bodies if getDistance(body.position,center) > size / 2]
    #
    # for escapee in escapes:
    #     bodies.remove(escapee)

    updates = [(body,calcAcceleration(body,getForces(body,bodies))) for body in bodies if not body.fixed]

    for update in updates:
        (body,aceleration) = update
        addPlace(body)
        pos = body.position
        velocityVector = body.velocity
        body.position = vectorSum([velocityVector,aceleration],pos)
        body.velocity = vectorSum([velocityVector,velocityVector,aceleration,aceleration],pos)

def fitWidth(text,space):
    font_scale = 10
    font = cv2.FORMATTER_FMT_DEFAULT
    scale = font_scale
    (txt_w,txt_h),baseline = cv2.getTextSize(text, font, scale, 1)
    while txt_w > space:
        scale -= 0.05
        (txt_w,txt_h),baseline = cv2.getTextSize(text, font, scale, 1)
    return scale,txt_h + baseline

def fitHeight(text,space):
    font_scale = 10
    font = cv2.FORMATTER_FMT_DEFAULT
    scale = font_scale
    txt_w,txt_h = cv2.getTextSize(text, font, font_scale, 1)[0]
    while txt_h > space:
        scale -= 0.05
        txt_w,txt_h = cv2.getTextSize(text, font, scale, 1)[0]
    return scale,txt_w

def Label(image,text,pos,width,color):
    font = cv2.FORMATTER_FMT_DEFAULT
    scale,height = fitWidth(text,width)
    cv2.putText(image,text,pos,font,scale,color,1)
    pos[1] += height

def plotBody(image,body):
    pos = body.position
    Opos = offsetPoint(pos)
    mass = body.mass
    velocity = body.velocity
    angle = getAngle(pos,velocity)
    speed = getDistance(pos,velocity)
    color = body.color
    Opos = intPoint(Opos)
    r = int(sqrt(mass))
    textPoint = [Opos[0] + r,Opos[1] + r]
    massText = 'mass :' + str(mass) + ' m unitz'
    Label(image,massText,textPoint,r*2,[20,20,90])
    velocitytext = 'speed : {:.2} px/t'.format(speed)
    Label(image,velocitytext,textPoint,r*2,[90,20,20])
    angletext = 'angle : {:10.2} degrees'.format(degrees(angle))
    Label(image,angletext,textPoint,r*2,[20,90,20])
    places = [intPoint(offsetPoint(place)) for place in body.places]
    places = array(places)
    if len(places) > 1:
        cv2.polylines(image,[places],False,[20,80,20])
    cv2.circle(image,Opos,r,color,2)

def intPoint(point):
    return [int(point[0]),int(point[1])]

def plotVector(image,body,vector,color,type=None):
    pos = body.position

    oVect = intPoint(offsetPoint(vector))
    oPose = intPoint(offsetPoint(pos))

    if type != None:
        length = getDistance(oPose,oVect)
        tagLength = int(sqrt(length)) * 2
        textPoint = [oPose[0] + tagLength,oPose[1] - tagLength]
        cv2.line(image,textPoint,oPose,color)
        velocitytext ='{}{:.2}'.format(type,length)
        Label(image,velocitytext,textPoint,length,color)

    cv2.arrowedLine(image,oPose,oVect,color,1)

def plotBodies(image,bodies):
    for body in bodies:
        plotBody(image,body)

        gravity = TotalGravity(body,bodies)

        velocity = body.velocity

        aceleration = calcAcceleration(body,gravity)

        pos = body.position

        plotVector(image,body,velocity,[50,50,100],'velocity')
        plotVector(image,body,gravity,[100,100,100],'gravity')
        plotVector(image,body,aceleration,[100,50,100],'aceleration')

def offsetPoint(pos):
    (x,y) = pos
    x += int(size / 2) - center[0]
    y += int(size / 2) - center[1]
    return (x,y)

def distToClosestBody(pos,bodies):
    return min([getDistance(pos,body.position) for body in bodies])

def getClosestBody(pos,bodies):
    d = distToClosestBody(pos,bodies)
    return [body for body in bodies if getDistance(pos,body.position) == d][0]

# def plotL12(image,body1,body2):

def get_cuberoot(x):
    if x < 0:
        x = abs(x)
        cube_root = x**(1/3)*(-1)
    else:
        cube_root = x**(1/3)
    return cube_root

def L1(body1,body2):
    M1 = body1.mass
    M2 = body2.mass
    P1 = body1.position
    P2 = body2.position
    R = getDistance(P1,P2)
    # i guess this is an aproxomation?
    r = R * get_cuberoot(M2/(3 * M1))
    distanceFromCom = (M1 / (M1 + M2) * R - r)
    centerOfMass = CenterOfMass([body1,body2])
    angle = getAngle(P1,P2)
    point = polarConversion(angle,centerOfMass,distanceFromCom)
    return point

def plotL1(image,body1,body2):
    point = L1(body1,body2)
    if point != None:
        plotsymbol(image,point,6,[[20,90,90],[90,90,20]],1)

def plotL2(image,body1,body2):
    pass
    # point = L2(body1,body2)
    # plotsymbol(image,point,10,[[100,20,100],[20,40,20]],3)

def plotLPoints(image,bodies):
    for body1 in bodies:
        for body2 in bodies:
            if body1.mass >= body2.mass and oneWay(body1,body2,bodies):
                plotL1(image,body1,body2)
                # plotL2(image,body1,body2)

def inAnyBody(point,bodies):
    return any(inBody(point,body) for body in bodies)

def inBody(point,body):
    p = body.position
    r = sqrt(body.mass)
    d = getDistance(point,p)
    return d < r

def plotGravityField(image,bodies):
    step = int(size/30)
    top = int(center[1]) - halfSize
    bottom = int(center[1]) + halfSize
    left = int(center[0]) - halfSize
    right = int(center[0]) + halfSize
    for y in range(top,bottom,step):
        for x in range(left,right,step):
            virtualBody = VirtualParticle((x,y))
            gravity = TotalGravity(virtualBody,bodies)
            if not (inAnyBody(gravity,bodies) or inAnyBody((x,y),bodies)):
                plotVector(image,virtualBody,gravity,[100,100,100])

def VirtualParticle(point):
    return createBody(point,100,0,0,[0,0,0])

def pointGravity(point,bodies):
    return getDistance(point,TotalGravity(VirtualParticle(point),bodies))

def SurfaceGravity(body):
    m = body.mass
    r = int(sqrt(m))
    g = G * (m / r)
    return g

def plotGravityGradiant(image,bodies):
    step = 20
    halfStep = int(step/2)
    Maxg = max([SurfaceGravity(body) for body in bodies])
    top = int(center[1]) - halfSize
    bottom = int(center[1]) + halfSize
    left = int(center[0]) - halfSize
    right = int(center[0]) + halfSize
    for y in range(top,bottom,step):
        for x in range(left,right,step):
            magnitude = pointGravity((x,y),bodies)
            (a,b) = intPoint(offsetPoint((x,y)))
            value = (magnitude / Maxg) * 255
            cv2.rectangle(image,(a-halfStep,b-halfStep),(a+halfStep,b+halfStep),[value,value,value],-1)

def section(offset,i,point,WA,r):
    return [intPoint(polarConversion(radians(j + offset),point,r)) for j in range(i * WA,(i + 1) * WA + 1)] + [point]

def plotsymbol(image,point,r,colors,NS):
    point = intPoint(offsetPoint(point))
    NC = len(colors)
    SA = int(360 / NS)
    WA = int(SA / NC)
    polys = [[section(SA * j,i,point,WA,r)for j in range(NS)] for i in range(NC)]
    for i in range(len(polys)):
        poly = polys[i]
        color1 = colors[i]
        color2 = colors[(i+1) % NC]
        poly = array(poly)
        cv2.fillPoly(image,poly,color1)
        cv2.polylines(image,poly,True,color2)

def plotCenterOfMass(image,bodies):
    COM = CenterOfMass(bodies)
    image = plotsymbol(image,COM,10,[[20,20,20],[10,50,50]],2)

def random_color():
    r = randint(50, 255)
    g = randint(50, 255)
    b = randint(50, 255)
    return (b, g, r)

def plotGrid(image):
    top = int(center[1]) - halfSize
    bottom = int(center[1]) + halfSize
    left = int(center[0]) - halfSize
    right = int(center[0]) + halfSize

    for x in range(left,right):
        if x % 30 == 0:
            offset = int(center[0])
            number = (halfSize + x - offset) * (halfSize - x + offset)
            margin = int(halfSize - sqrt(number))
            point1 = intPoint(offsetPoint((x,top + margin)))
            point2 = intPoint(offsetPoint((x,bottom - margin)))
            cv2.line(image,point1,point2,[90,90,90])

    for y in range(top,bottom):
        if y % 30 == 0:
            offset = int(center[1])
            number = (halfSize + y - offset) * (halfSize - y + offset)
            margin = int(halfSize - sqrt(number))
            point1 = intPoint(offsetPoint((left + margin,y)))
            point2 = intPoint(offsetPoint((right - margin,y)))
            cv2.line(image,point1,point2,[90,90,90])

    point = intPoint(offsetPoint(center))
    cv2.circle(image,point,halfSize,[0,0,40],2)

def createFrame(bodies):
    global center
    center = CenterOfMass(bodies)

    frame = np.zeros((size,size,3),dtype=np.uint8)

    plotGravityGradiant(frame,bodies)
    # plotGravityField(frame,bodies)
    plotBodies(frame,bodies)
    # plotLPoints(frame,bodies)
    plotCenterOfMass(frame,bodies)
    # plotGrid(frame)
    return frame

def createGif(bodies):
    windowName = 'gif'
    maxFrames = 300
    first = None
    image_count = 0
    frames = []
    while True:
        updateBodies(bodies)
        frame = createFrame(bodies)
        cv2.imshow(windowName,frame)
        cv2.waitKey(1)
        frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        equal = np.array_equal(first,frame)
        if equal or image_count > maxFrames:
            print("Saving GIF file")
            with imageio.get_writer("orbit.gif", mode="I") as writer:
                for idx, frame in enumerate(frames):
                    print("Adding frame to GIF file: ", idx + 1)
                    writer.append_data(frame)
            return
        else:
            frames.append(frame)
            image_count += 1
        if np.array_equal(first,None):
            first = frame

size = 900
halfSize = int(size/2)

offset = int(size / 8)

G = size / 100

color = [50,30,20]

bodies = []

center = (0,0)



# bodies.append(createBody((0,0),1500,.8,-90,color,True))

bodies.append(createBody((0,0),3000,.33,90,color))
bodies.append(createBody(( offset,0),100,16,-90,color))
bodies.append(createBody((2 * offset,0),100,12,-90,color))

# createGif(bodies)

while True:

    while len(bodies) < 5:
        bodies += [randBody()]

    # sleep(.03)
    frame = createFrame(bodies)
    cv2.imshow('space',frame)
    cv2.waitKey(1)
    updateBodies(bodies)

    # todo:
    # reference center of mass for point ofset to keep it centered done, somehow
    # inalastic collisions...done
    # lagrange points... good luck
    # senarios
    # menue options with buttons and sliders
