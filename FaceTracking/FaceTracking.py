import time
# please install manually these dependencies below
import cv2
import numpy as np
from djitellopy import tello

alive = True
width, height = 360,240 #a méretek a megjelenítéshez
prewErrorRotate = 0 #az előző hiba a forduláshoz
prewErrorUD = 0 # a fel és le mozgás korábbi hibája
prewErrorBFmin = 0 # az előző minmális hiba az előre hátra mozugáshoz
prewErrorBFmax = 0 # az előző maximális hiba az előre hátra mozugáshoz
#prewErrorBF = 0 # az előző hiba az előre és hátra repüléshez
fbRange = [6000,8000]   # ez lesz az az intervallum, amiben elfogadjuk a az arcterület nagyságát, ha ettől nagyobb, vagy
                        # kisebb, akkor előre vagy hátra mozgunk
pid = [0.4,0,0.4] #propocionális, integrál, derivál
pid2 = [0.01,0,0.01]


# a tello objektum példánya
drone = tello.Tello()
# csatlakozás a drónhoz WiFi-vel
drone.connect()
# a drón kamerájáról érkező képek stream-elése
drone.streamon()
drone.takeoff()
drone.send_rc_control(0,0,25,0)
time.sleep(1)


#capture = cv2.VideoCapture(0) #a számítógép webkamerájáról vesszük most a folyamatos frameket # test with camera 1/2.


"""
Ez a metódus felismeri az arcot egy előre megírt cascade segítségével.

@return img, [faceListCenter[index],faceListArea[index]]: a felismert arcok közül a legnagyobb, valamint  az arc 
                                                            területe és középpontja
@param img: egy kép
"""
def findFace(img):

    faceCascade = cv2.CascadeClassifier("Resources/haarcascade_frontalface_default.xml")
    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) # szürkeskálássá alakítjuk a színes képet
    faces = faceCascade.detectMultiScale(imgGray,1.2,8) #detektálja az arcot és visszadaja
                                                        #az arcokat befoglalaó terület az x,y koordinátáit,
                                                        # valamint a terület magasságát és a szélsségét

    faceListCenter = [] #az összes arcot tartalmazó kép
    faceListArea = [] #az összes tényleges arcterület <- a legnagyobb arcok kerülnek ide, azt feltételezve,
                    # hogy ezeket kell trackerlni


    for (x,y,w,h) in faces:

        cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2) # az arc köré egy piros 2 vastag négyzetet

        # megkeressük a  képek közepét és integerré alakítjuk a koordinátát
        centerX = x + w // 2
        centerY = y + h // 2

        area = w * h # kiszámítjuk az arc területet a képen

        cv2.circle(img,(centerX,centerY),5,(0,255,0),cv2.FILLED)# kirajzoljuk a kép közepét

        # eltároljuk az arcokat befoglaló négyzetek közepét és a négyzet területét
        faceListCenter.append([centerX,centerY])
        faceListArea.append(area)

    # ha van már elemünk a lsitában
    if len(faceListArea) != 0:
        index = faceListArea.index(max(faceListArea))   # akkor adjuk vissza a képen detektálható arcok közül a legnagyobb
                                                        # területűt

        return img, [faceListCenter[index],faceListArea[index]] # és visszatérünk a legnagyobb arccal, mint kép, illetve
                                                                # a hozzá kapcsolódó területtel és középponttal
    # ha nincs arc a képen
    else:
        return img, [[0,0],0] #térjünk vissza nulla értékekkel

"""
Ez a függvény a felismert arc követésének megvalósításához ad ki poarancsokat a drónnak.

@return errorRotate: a felismert arc középpontja és a kép középpontjának távolsága az x tengely mentén
@return errorUD:  a felismert arc középpontja és a kép középpontjának távolsága az y tengely mentén
@return errorFBmin: a befoglaló négyzet területe és a minimális elfogadási terület különbsége
@return errorFBmax: a befoglaló négyzet területe és a maximális elfogadási terület különbsége

@param drone: a Tello objektum, ami a parancsokat fogadja
@param imgInfo: az arc területe és középpontja
@param width: a kép szélessége
@param height: a lép magassága
@param pidRotation: a forgatáshoz és fel-le repüléshez használt PID együtthatók
@param pidFB: az előre-hátra repüléshez használt PID együtthatók
@param prewErrorRotate: az előző errorRotate
@param prewErrorUD: az előző errorUD
@param prewErrorFBmin: az előző errorFBmin
@param prewErrorFBmax: az előzőErrorFBmax
"""
def trackFace(drone, imgInfo, width,height, pidRotation, pidFB, prewErrorRotate, prewErrorUD, prewErrorFBmin, prewErrorFBmax):

    area = imgInfo[1]
    x,y = imgInfo[0]

    # itt a forgatáshoz szülkséges dolgok vannak
    errorRotate = x - width//2 # ez adja meg az objektum távolságát a középponttól széltében (set point és process variable külöönbsége)
    errorUD = y - height//2 # ez adja meg az objektum távolságát a középponttól magasságban (set point és process variable külöönbsége)
    errorFBmax = 0
    errorFBmin = 0

    # a különböző mozgási irányok sebességének  beállítása/inicalizálása
    speedOfRotation = pidRotation[0] * errorRotate + pidRotation[2] * (errorRotate - prewErrorRotate) #a  sebességet a pid segítségével számoljuk.
                                                                            # az integrált nem használjuk

    speedOfRotation = int(np.clip(speedOfRotation,-100,100)) # a sebesség -100 és 100 közé essen csak

    speedOfUD = pidRotation[0] * errorUD + pidRotation[2] * (errorUD - prewErrorUD) #a  sebességet a pid segítségével számoljuk.
                                                                            # az integrált nem használjuk
    speedOfUD = int(np.clip(speedOfUD,-100,100))

    speedOfFB = 0
    # itt az elfogadási intervallum területébe esés ellenőrzése történik
    # egész addig nincs előer-hátra reptetés, amíg nagyobb a befoglaló négyzet tetülete, mint az intervallum alsó
    # határa és kisebb, mint az intervallum eflső határa
    if area < fbRange[0] and area != 0:
        errorFBmin = fbRange[0] - area
        speedOfFB = pidFB[0] * errorFBmin + pidFB[2] * (errorFBmin - prewErrorFBmin)
        errorFB = errorFBmin
    elif area > fbRange[1]:
        errorFBmax = fbRange[1] - area
        speedOfFB = speedOfFB = pidFB[0] * errorFBmax + pidFB[2] * (errorFBmax - prewErrorFBmax)
        errorFB = errorFBmax

    speedOfFB = int(np.clip(speedOfFB,-20,20))

    # itt biztosítom, hogy csak akkor legyen irányváltás és hiba generálás, ha van felismert arc a képen
    #ellenkező esetben ne legyen se hiba, se semmilyen irányváltoztatás
    if x == 0:
        speedOfRotation = 0
        errorRotate = 0

    if y == 0:
        speedOfUD = 0
        errorUD = 0

    if area == 0:
        speedOfFB = 0
        errorFBmin = 0
        errorFBmax = 0
        speedOfUD = 0
        errorUD = 0


    drone.send_rc_control(0,speedOfFB,(speedOfUD)*(-1),speedOfRotation) # itt küldöm el a kiszámított írányítást a drónnak
    return (errorRotate,errorUD,errorFBmin,errorFBmax) # visszatérünk a hibákal

if __name__ == '__main__':

    # itt vesszük le a folyamatos frameket
    while alive:
        #_,img = capture.read() # test with camera 2/2.
        img = drone.get_frame_read().frame
        img = cv2.resize(img,(width,height))
        img, imgInfo = findFace(img)

        prewErrorRotate, prewErrorUD,prewErrorBFmin, prewErrorBFmax = trackFace(drone, imgInfo, width, height, pid,pid2,
                                                                               prewErrorRotate,
                                                                               prewErrorUD, prewErrorBFmin, prewErrorBFmax)


        cv2.imshow("Drone",img)
        cv2.waitKey(1)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            drone.streamoff()
            drone.land()
            alive = False
