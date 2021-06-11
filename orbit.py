import cv2
import imageio
import numpy as np
from numpy import array
from math import sin, cos, tan, pi, sqrt, ceil, log, degrees, radians, atan2
from time import time_ns, sleep
from random import randint

def randBody():
    global size
    x = randint(-int(size/4),int(size/4))
    y = randint(-int(size/4),int(size/4))
    pos = (x,y)
    distance = getDistance(pos,(0,0))
    mass = randint(1,10)
    speed = ((size / 10) / distance) * (randint(10,30) / 10)
    angle = degrees(atan2(y,x)) + 90
    color = [100,100,100]
    return createBody(pos,mass,speed,angle,color)

def createBody(pos,mass,speed,angle,color,fixed=False):
    velocity = polarConversion(radians(angle),pos,speed)
    body = {
        'position':pos,
        'mass':mass,
        'velocity':velocity,
        'color':color,
        'fixed':fixed,
        'places':[]
    }
    return body

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
    pos = body['position']
    mass = body['mass']
    magnitude = getDistance(pos,forceVector)
    acelerationScaler = magnitude/mass
    acelerationVector = createVector(pos,forceVector,acelerationScaler)
    return acelerationVector

def gravity(body1,body2):
    pos1 = body1['position']
    pos2 = body2['position']
    mass1 = body1['mass']
    mass2 = body2['mass']
    r = getDistance(pos1,pos2)
    force=(G*mass1*mass2)/r**2
    vector = createVector(pos1,pos2,force)
    return vector

def TotalGravity(body,bodies):
    return vectorSum([gravity(body,body2) for body2 in bodies if body2['position'] != body['position']],body['position'])

def getForces(body,bodies):
    pos = body['position']
    gravity = TotalGravity(body,bodies)
    # posible other forces
    forceSum = vectorSum([gravity],pos)
    return forceSum

maxPos = 2200

def massRadius(body):
    return int(sqrt(body['mass']))

def CenterOfMass(body1,body2):
    m1 = body1['mass']
    m2 = body1['mass']
    pos1 = body1['position']
    pos2 = body2['position']
    centerX = (m1 * pos1[0] + m2 * pos2[0]) / (m1 + m2)
    centerY = (m1 * pos1[1] + m2 * pos2[1]) / (m1 + m2)
    return (centerX,centerY)

def combineBody(body1,body2,bodies):
    if not (body1['fixed'] or body2['fixed']):
        pos1 = body1['position']
        pos2 = body2['position']
        v1 = body1['velocity']
        m1 = body1['mass']
        v2 = body1['velocity']
        m2 = body1['mass']
        mT = m1 + m2
        COM = CenterOfMass(body1,body2)
        p1 = vectorMultiplacation(COM,v1,m1)
        p2 = vectorMultiplacation(COM,v2,m2)
        pT = vectorSum([p1,p2],(COM))

        print(pT)

        vT = vectorMultiplacation(COM,pT,1/mT)
        body3 = {
            'position':COM,
            'mass':mT,
            'velocity':vT,
            'color':[50,30,20],
            'fixed':False,
            'places':[]
        }
        bodies.append(body3)
        if body1['mass'] >= body2['mass']:
            body3['places'] = body1['places']
        else:
            body3['places'] = body2['places']
        bodies.remove(body1)
        bodies.remove(body2)
    elif body1['fixed']:
        bodies.remove(body2)
    else:
        bodies.remove(body1)


def updateBodies(bodies):

    accelerations = [calcAcceleration(body,getForces(body,bodies)) for body in bodies if not body['fixed']]

    index = 0

    stop = False

    for body in bodies:
        for body2 in bodies:
            if body != body2:
                pos1 = body['position']
                pos2 = body2['position']
                if getDistance(pos1,pos2) < massRadius(body) + massRadius(body2):
                    combineBody(body,body2,bodies)
                    stop = True
                    break

        if stop:
            break

        if not body['fixed']:

            aceleration = accelerations[index]

            velocityVector = body['velocity']

            pos = body['position']

            if getDistance(pos,(0,0)) > size / 2:
                bodies.remove(body)
                break

            places = body['places']

            places.append(intPoint(offsetPoint(pos)))

            if len(places) > maxPos:
                places.pop(0)

            newVelocity = vectorSum([velocityVector,velocityVector,aceleration,aceleration],pos)

            body['position'] = vectorSum([velocityVector,aceleration],pos)

            body['velocity'] = newVelocity

            index += 1

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
    pos = body['position']
    Opos = offsetPoint(pos)
    mass = body['mass']
    velocity = body['velocity']
    angle = getAngle(pos,velocity)
    speed = getDistance(pos,velocity)
    color = body['color']
    Opos = intPoint(Opos)
    r = int(sqrt(mass))

    textPoint = [Opos[0] + r,Opos[1] + r]

    massText = 'mass :' + str(mass) + ' m unitz'
    Label(image,massText,textPoint,r*2,[20,20,90])

    velocitytext = 'speed : {:.2} px/t'.format(speed)
    Label(image,velocitytext,textPoint,r*2,[90,20,20])

    angletext = 'angle : {:.2} degrees'.format(degrees(angle))
    Label(image,angletext,textPoint,r*2,[20,90,20])

    points = array(body['places'])
    if len(points) > 1:
        cv2.polylines(image,[points],False,[20,80,20])
    cv2.circle(image,Opos,r,color,2)

def intPoint(point):
    return [int(point[0]),int(point[1])]

def plotVector(image,body,vector,color,type=None):
    pos = body['position']

    oVect = intPoint(offsetPoint(vector))
    oPose = intPoint(offsetPoint(pos))


    if type != None:
        length = getDistance(oPose,oVect)
        textPoint = [oPose[0]+int(length),oPose[1]-int(length)]
        cv2.line(image,textPoint,oPose,color)
        velocitytext ='{}{:.2}'.format(type,length)
        Label(image,velocitytext,textPoint,length,color)

    cv2.arrowedLine(image,oPose,oVect,color,1)

def drawBodies(bodies):
    global size
    space = np.zeros((size,size,3),dtype=np.uint8)
    for body in bodies:
        plotBody(space,body)

        gravity = TotalGravity(body,bodies)

        velocityVector = body['velocity']

        aceleration = calcAcceleration(body,gravity)

        # bigvector = vectorMultiplacation(pos,vector,1)

        plotVector(space,body,velocityVector,[50,50,100],'velocity')
        plotVector(space,body,gravity,[100,100,100],'gravity')
        plotVector(space,body,aceleration,[100,50,100],'aceleration')

    return space

def createGif(bodies):
    windowName = 'gif'
    maxFrames = 300
    first = None
    image_count = 0
    frames = []
    while True:
        updateBodies(bodies)
        frame = drawBodies(bodies)
        frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        cv2.imshow(windowName,frame)
        cv2.waitKey(1)
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

def offsetPoint(pos):
    (x,y) = pos
    x += int(size / 2)
    y += int(size / 2)
    return (x,y)

def minDistancefromBodies(pos,bodies):
    return min([getDistance(pos,body['position']) for body in bodies])

def inLine(pos1,pos2,pos3):
    return getDistance(pos1,pos2) + getDistance(pos2,pos3) < getDistance(pos1,pos3) + .001

def plotL1(image,bodies):
    step = int(size/90)
    halfStep = int(step/2)
    done = []
    for body1 in bodies:
        for body2 in bodies:
            if bodies.index(body2) > bodies.index(body1):
                pos1 = body1['position']
                pos2 = body2['position']
                pos1 = intPoint(pos1)
                pos2 = intPoint(pos2)
                dx = max(pos1[0],pos2[0]) - min(pos1[0],pos2[0])
                dy = max(pos1[1],pos2[1]) - min(pos1[1],pos2[1])
                for j in range(min(pos1[0],pos2[0]),max(pos1[0],pos2[0])):
                    for i in range(min(pos1[1],pos2[1]),max(pos1[1],pos2[1])):
                        if inLine(pos1,(j,i),pos2):
                            point = (j,i)
                            minDistance = minDistancefromBodies(point,bodies)
                            virtualBody = createBody(point,100,0,0,color)
                            gravity = TotalGravity(virtualBody,bodies)
                            magnitude = getDistance(point,gravity)
                            if magnitude < minDistance:
                                if magnitude < .1:
                                    (a,b) = intPoint(offsetPoint(point))
                                    cv2.rectangle(image,(a-halfStep,b-halfStep),(a+halfStep,b+halfStep),(20,90,90),-1)
    return image

def plotGravityField(image,bodies):
    step = int(size/90)
    halfStep = int(step/2)
    for y in range(-size,size,step):
        for x in range(-size,size,step):
            pos = (x,y)
            minDistance = minDistancefromBodies(pos,bodies)
            virtualBody = createBody(pos,100,0,0,color)
            gravity = getForces(virtualBody,bodies)
            magnitude = getDistance(pos,gravity)
            if magnitude < minDistance:
                if magnitude < .1:
                    (a,b) = intPoint(offsetPoint(pos))
                    cv2.rectangle(image,(a-halfStep,b-halfStep),(a+halfStep,b+halfStep),(20,20,90),-1)
                else:
                    plotVector(image,virtualBody,gravity,[100,100,100])
    return image


size = 600

Offset = int(size / 6)
smallOffset = int(size / 5)

G = .1

color = [50,30,20]

bodies = []

# bodies.append(createBody((0,0),1500,.1,-90,color,True))


bodies.append(createBody((-smallOffset,0),4000,0,0,color))
bodies.append(createBody(( smallOffset,0),4000,0,0,color))


# createGif(bodies)

while True:

    # if len(bodies) < 5:
        # bodies += [randBody()]
    sleep(.03)
    frame = drawBodies(bodies)
    # frame = plotL1(frame,bodies)
    # frame = plotGravityField(frame,bodies)
    cv2.imshow('space',frame)
    cv2.waitKey(1)
    updateBodies(bodies)
