# coding=utf-8
import sys
from xml.sax import make_parser
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import numpy as np
import sys

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    global offset
    global dif
    
    
    dif = 0
    offset = [0,0]
    
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.count = 0
        self.viraEsq = 0
        self.viraDir = 0
        self.maxLeft = 0
        self.maxRight = 0
        self.posX = 0 
        self.posY = 0
        self.previousGps = [0,0]
        self.contadorCiclos = 0
        self.firstPosX = 0
        self.firstPosY = 0
        self.coordinates = np.array([(x,y,check) for x in range(56) for y in range(28) for check in range(1)]) #array com coordenadas do mapa

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True)
                self.wander()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander()
            

    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        lin = 0.14

        print("sensor parede da frente:", (self.measures.irSensor[center_id])) 
        # print("sensor parede da direita:", (self.measures.irSensor[right_id]))
        # print("sensor parede da esquerda:", (self.measures.irSensor[left_id]))
        # print("\n")
        #print(self.measures.compass)

        if self.contadorCiclos == 0:
            global dx_ant
            global dif
            dif = 0
            dx_ant = 1.0
            self.contadorCiclos+=1
            self.firstPosX =self.measures.x-28 #tirar o 3 para ficar caso geral
            self.firstPosY =self.measures.y-14 #  //  o 11       //        //
            self.previousGps = [self.measures.x - self.firstPosX, self.measures.y - self.firstPosY]
            self.driveMotors(0.15, 0.15)
            


        self.posX = self.measures.x-self.firstPosX #variavel que guarda a coordenada 
        self.posY = self.measures.y-self.firstPosY #sem ter que se estar sempre a fazer a conta
        print("coordenada x: ", self.posX)
        print("coordenada y: ", self.posY)

        if self.count == 5:
            self.count = 0
            self.viraEsq = 0
            self.viraDir = 0

        elif self.viraEsq == 1:
            self.driveMotors(-0.13, 0.13)   #virar à esquerda
            print("vira à esquerda")
            print("Viragem numero:", self.count)
            self.count +=1

        elif self.viraDir == 1:
            self.driveMotors(0.13, -0.13)   #virar à direita
            print("vira à direita")
            print("Viragem numero:", self.count)
            self.count +=1
        else:
            
            print("gps "+ str(self.posX)+ " " + str(self.posY))
            print(self.measures.compass)
            d_x = self.posX - self.previousGps[0]
            d_y = self.posY - self.previousGps[1]

            if(self.measures.compass >=-3 and self.measures.compass <=3) or ((self.measures.compass <=-179 and self.measures.compass >=-180) and (self.measures.compass <=180 and self.measures.compass >=170)):   #Encontra-se na horizontal 

                if ( d_x < 2.0):  #Anda para algum na horizontal
                    
                    if (self.measures.compass >= -2 and self.measures.compass <=0) or (self.measures.compass>= 179 and self.measures.compass<=180):   #esta ligeiramente inclinado para a direita
                        self.driveMotors(0.13, 0.15)
                    elif (self.measures.compass <=2 and self.measures.compass >= 0) or (self.measures.compass>= -180 and self.measures.compass<=-179):  #esta ligeiramente inclinado para a esquerda 
                        self.driveMotors(0.15, 0.13)
                    else:
                        self.driveMotors(0.15, 0.15)
                        print("avança "+ str(self.posX)+ " " + str(self.posY))

                else:   #encontra-se a meio de uma celula e atualiza o previousGpsd_x_ant = d_x
                    dif = 2.0 - d_x
                    print("D_X ",d_x) 
                    self.previousGps = [self.posX + dif, self.posY]   #atualiza a posição atual
                    self.driveMotors(-0.15, -0.15)
                    print("stop***********************************")
                    if self.measures.irSensor[center_id] < 1/0.85:     #Tem a possibilidade de ir em frente
                        print("pode ir em frente")
                        self.driveMotors(0.15,0.15)  
                    elif(self.measures.irSensor[left_id])< 1/0.7:     #Tem a possibilidade de ir para a esquerda
                        print("pode virar á esquerda")
                        self.viraEsq = 1
                    elif (self.measures.irSensor[right_id])< 1/0.7: #Tem a possibilidade de ir para a direita
                        self.viraDir = 1
                        print("pode virar á direita")
            elif (self.measures.compass >=60 and self.measures.compass <=120) or (self.measures.compass >=-87 and self.measures.compass <=-93):  #encontra-se na vertical   
                #TODO

                
                



                if ( d_y < 2.0):  #Anda para algum na vertical
                    
                    self.driveMotors(0.15, 0.15)
                    print("avança na vertical------------------------------------ "+ str(self.posX)+ " " + str(self.posY))
                    

                else:   #encontra-se a meio de uma celula e atualiza o previousGpsd_x_ant = d_x
                    dif = 2.0 - d_y 
                    self.previousGps = [self.posX, self.posY + dif]   #atualiza a posição atual (trocaste aqui a subtraçao da dif)
                    self.driveMotors(-0.15, -0.15)
                    print("stop***********************************")
                    #TODO FAZER A CENA DE ACERTAR COM A BUSSOLA
                    if self.measures.irSensor[center_id] < 1/0.75:     #Tem a possibilidade de ir em frente
                        print("pode ir em frente")
                        self.driveMotors(0.15,0.15)  
                    elif(self.measures.irSensor[left_id])< 1/0.7:     #Tem a possibilidade de ir para a esquerda
                        print("pode virar á esquerda")
                        self.viraEsq = 1
                    elif (self.measures.irSensor[right_id])< 1/0.7: #Tem a possibilidade de ir para a direita
                        self.viraDir = 1
                        print("pode virar á direita")
            
            #TODO Se estiver num beco voltar a trás
            #print(previousGps, self.measures.compass)

            #if self.measures.irSensor<=2 :  #Pode andar em frente


       
        
        
        #DEPOIS EXECUTAR A FUNÇÃO FINISH

        #posicao central do 55,27 e 28,14
        #x,y, e check -> check a 0 quer dizer que nao passou na celula, check a 1 quer dizer que passou
        

        indInicial = int(self.posX)*27+int(self.posX) #valor de x -> posiçao no array
        indFinal = indInicial + 27
        intervalo = self.coordinates[indInicial:indFinal+1] #intervalo de valores possiveis de y, correspondentes a esse valor de x
        for i in range(len(intervalo)):
            if intervalo[i][1] == int(self.posY): #se o valor for igual á coordenada y 
                intervalo[i][2] = 1 #esse valor adquire o valor 1, quer dizer que ja foi visitado
                print("check:", intervalo[i])
        
    


class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
