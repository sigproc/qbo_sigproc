#!/usr/bin/env python

#Simple Xbox360 joypad test program by fg295

#import ROS dependencies 
import roslib; roslib.load_manifest('qbo_laser_slam')
import rospy
import yaml
import math
import Image
 
#import and init pygame
import sys
import pygame; pygame.init()

#Import ROS message to publish movements
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Joy

#create the window
width = 640
height = 480
aspect = width/height
window = pygame.display.set_mode((width, height))
pygame.display.set_caption('Q.Bo Exploration simulator by SifuF') 

#globals
qboX = 20
qboY = 0
qboXtarget = qboX
qboYtarget = qboY
mapX = 0
mapY = 0 
targetIterMax = 500
targetIter = targetIterMax
qboUP = 0
qboDOWN = 0
qboLEFT = 0
qboRIGHT = 0
mapMoving = 0
mouseStart = (0,0)
unknown = 205, 205, 205, 255
floor = 254, 254, 254, 255
wall = 0, 0, 0, 255
im = 0
pix = 0
mapImage = 0
qboImage = 0
resolution = 0
originX = 0
originY = 0
mapLengthToDisplay = 35 #meters

def renderScene():
    global qboX, qboY, originX, originY, mapX, mapY, qboXtarget, qboYtarget
    global qboLEFT, qboRIGHT, qboUP, qboDOWN, mapMoving, mouseStart 
    global targetIter, targetIterMax

    window.fill((0,0,0))
    
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                qboLEFT = 1
            if event.key == pygame.K_RIGHT:
                qboRIGHT = 1
            if event.key == pygame.K_UP:
                qboUP = 1
            if event.key == pygame.K_DOWN:
                qboDOWN = 1
            if event.key == pygame.K_a:
                mapX -= 100 
                qboX -= 100
            if event.key == pygame.K_s:
                mapY += 100
                qboY += 100
            if event.key == pygame.K_d:
                mapX += 100
                qboX += 100
            if event.key == pygame.K_w:
                mapY -= 100
                qboY -= 100
        
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_LEFT:
                qboLEFT = 0
            if event.key == pygame.K_RIGHT:
                qboRIGHT = 0
            if event.key == pygame.K_UP:
                qboUP = 0
            if event.key == pygame.K_DOWN:
                qboDOWN = 0

        if event.type == pygame.MOUSEBUTTONDOWN:  
            if event.button == 1:
                targetIter = 0
                pos = event.pos
                qboXtarget = pos[0] - qboImage.get_width()/2
                qboYtarget = pos[1] - qboImage.get_height()/2

            if event.button == 2:
                mapMoving = 1
                mouseStart = pygame.mouse.get_pos()
            
            if event.button == 3:
                findFrontiers()
                updateMap()

        if event.type == pygame.MOUSEBUTTONUP:  

            if event.button == 2:
                pos = mapMoving = 0
                
                


    if qboLEFT != 0:
        qboX -= 0.5
    if qboRIGHT != 0:
        qboX += 0.5
    if qboUP != 0:
        qboY -= 0.5
    if qboDOWN != 0:
        qboY += 0.5



    if mapMoving !=0:
        mpos = pygame.mouse.get_pos()
        mapX = mapX + (mpos[0] - mouseStart[0])
        qboX = qboX + (mpos[0] - mouseStart[0])
        qboXtarget = qboXtarget + (mpos[0] - mouseStart[0])
        mapY = mapY + (mpos[1] - mouseStart[1])
        qboY = qboY + (mpos[1] - mouseStart[1])
        qboYtarget = qboYtarget + (mpos[1] - mouseStart[1])
        mouseStart = mpos
    elif targetIter < targetIterMax:
        qboX = qboX + (qboXtarget-qboX)*targetIter/targetIterMax
        qboY = qboY + (qboYtarget-qboY)*targetIter/targetIterMax

        targetIter+=0.5
        if targetIter > targetIterMax:
            targetIter = targetIterMax

    window.blit(mapImage, (mapX,mapY)) 
    window.blit(qboImage, (qboX,qboY)) 
    pygame.display.flip() 


def findFrontiers():
    
    pygame.draw.circle(window, (0, 255, 0), (40, 40), 20) 
    for i in range(0, im.size[0]):
        for j in range(0, im.size[1]):
            if pix[i,j] == floor:
                if pix[i+1,j] == unknown or pix[i,j+1] == unknown:
                    pix[i,j] = 255, 0, 0, 255 

    im.save("/home/sifuf/cued-masters/src/qbo_sigproc/qbo_laser_slam/src/simulator/ima.png","PNG")




def listener():

    #rospy.init_node('exploration_simulator', anonymous=True)
    #rospy.Subscriber("/joy", Joy, callback)
  
    while(1):
        renderScene()
        
    
 
    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

def updateMap():
    global mapImage, resolution, originX, originY, mapLengthToDisplay, mapX, mapY
    mapImage = pygame.image.load('/home/sifuf/cued-masters/src/qbo_sigproc/qbo_laser_slam/src/simulator/ima.png')
    mapAspect = mapImage.get_width()/mapImage.get_height()     
    mapWidthInPixels = mapLengthToDisplay/resolution #required visible pixel width
    l = mapImage.get_width() - mapWidthInPixels
    lDash = l*width/mapWidthInPixels
    scaledMapWidth = width + lDash
    scaledQboWidth = width*1/mapLengthToDisplay
    mapImage = pygame.transform.scale(mapImage, (int(scaledMapWidth), int(scaledMapWidth/mapAspect)))
    #mapX = -scaledMapWidth/2 + 200
    #mapY = -scaledMapWidth/(2*mapAspect) + 300
    window.blit(mapImage, (mapX,mapY)) #draw map


def init():
    global im, pix, qboImage, mapImage, mapX, mapY, qboX, qboY, mapLengthToDisplay, resolution, originX, originY
    im = Image.open('/home/sifuf/cued-masters/src/qbo_sigproc/qbo_laser_slam/src/simulator/map8.pgm')
    im = im.convert('RGBA')
    pix = im.load()

    #initial layout
    stream = open('/home/sifuf/cued-masters/src/qbo_sigproc/qbo_laser_slam/src/simulator/map8.yaml', 'r')
    settings = yaml.load(stream)
    resolution = settings["resolution"]
    originX = settings["origin"][0]
    originY = settings["origin"][1]
    
    window.fill((0,0,0))
    
    mapImage = pygame.image.load('/home/sifuf/cued-masters/src/qbo_sigproc/qbo_laser_slam/src/simulator/map8.pgm')
    mapAspect = mapImage.get_width()/mapImage.get_height()     
    mapWidthInPixels = mapLengthToDisplay/resolution #required visible pixel width
    l = mapImage.get_width() - mapWidthInPixels
    lDash = l*width/mapWidthInPixels
    scaledMapWidth = width + lDash
    scaledQboWidth = width*1/mapLengthToDisplay
    mapImage = pygame.transform.scale(mapImage, (int(scaledMapWidth), int(scaledMapWidth/mapAspect)))
    mapX = -scaledMapWidth/2 + 200
    mapY = -scaledMapWidth/(2*mapAspect) + 300

    qboImage = pygame.image.load('/home/sifuf/cued-masters/src/qbo_sigproc/qbo_laser_slam/src/simulator/qbo.png')
    qboAspect = float(qboImage.get_width())/qboImage.get_height()  
    scaledQboWidth = width*1/mapLengthToDisplay
    qboImage = pygame.transform.scale(qboImage, (int(scaledQboWidth), int(scaledQboWidth/qboAspect)))
    
    window.blit(qboImage, (qboX,qboY)) #draw qbo
    window.blit(mapImage, (mapX,mapY)) #draw map
    pygame.display.flip() 



        
if __name__ == '__main__':
    init()
    listener()
