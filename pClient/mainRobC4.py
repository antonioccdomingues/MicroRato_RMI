# coding=utf-8
import math
import sys
from xml.sax import make_parser
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import numpy as np
import sys
import random

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    global deslocamentoX
    global deslocamentoY
    global previousPowerR
    global previousPowerL
    global newGPS

    global arrayPotencias
    arrayPotencias = [0.131, 0.131, 0.131, 0.131, 0.131 ] 

    global arrayPotenciasReverte
    arrayPotenciasReverte = [0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.131] 

    newGPS = [27, 13]   #coordenadas iniciais do gps
    previousPowerR = 0
    previousPowerL = 0
    deslocamentoX = 27
    deslocamentoY = 13
    
    
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.count = 0
        self.viraEsq = 0
        self.viraDir = 0
        self.reverte = 0
        self.countReverte = 0
        self.maxLeft = 0
        self.maxRight = 0
        self.posX = 0 
        self.posY = 0
        self.previousGps = [0,0]
        self.contadorCiclos = 0
        self.firstPosX = 0
        self.firstPosY = 0
        rows, cols = (27, 56)
        self.foo = " " #inicializa todos os espaços com " "
        self.coordinates = [[self.foo for x in range(rows)] for y in range(cols)] #cria um array bidimensional 56 colunas e 27 linhas
        self.mediaCoordenadas = 0
        
        self.orientacaoX = 0
        self.orientacaoY = 0
        

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

    def updatePreviousMotors(self, l, r):
        global previousPowerL
        global previousPowerR

        previousPowerL = (l + previousPowerL) /2
        previousPowerR = (r + previousPowerR) /2
        
            

    def wander(self):
        
        global deslocamentoX
        global deslocamentoY
        global arrayPotencias
        global newGPS
        global arrayPotenciasReverte

        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        lin = 0.14
        


        if self.contadorCiclos == 0: #PRIMEIRO CICLO
            
            self.updatePreviousMotors(0, 0) #no 1º ciclo a previousMotors é zero
            deslocamentoX = 27
            deslocamentoY = 13

            self.contadorCiclos+=1
            self.firstPosX =self.measures.x-27
            self.firstPosY =self.measures.y-14
            self.posX = 27
            self.posY = 13
            print(newGPS[0])
            if self.measures.compass <= 10 and self.measures.compass >= -10:
                self.coordinates[newGPS[0]+1][newGPS[1]]= "X"
            elif abs(self.measures.compass) <= 180 and abs(self.measures.compass) >= 170: 
                self.coordinates[newGPS[0]-1][newGPS[1]] = "X"

            #self.reverte = 1

            #self.previousGps = [self.measures.x - self.firstPosX, self.measures.y - self.firstPosY]
            self.previousGps = [27, 13]
        # with open('map.out', 'w') as outfile:
        #     result = [[self.coordinates[j][i] for j in range(len(self.coordinates))] for i in range(len(self.coordinates[0]))]
        #     for row in range(27):
        #         temp = result[row]
        #         result[row] = result[26-row]
        #         result
        #     for i in range(27):
        #         for j in range(55):
        #             outfile.write(self.coordinates[i][j])
        #             #print(self.coordinates[i][j])
        #         outfile.write("\n")
        #print("\n".join(["".join([x for x in row])for row in self.coordinates]))
        
        self.coordinates[27][13] = "I" #posicao inicial
        self.posX = self.measures.x - self.firstPosX #variavel que guarda a coordenada 
        self.posY = self.firstPosY - self.measures.y #sem ter que se estar sempre a fazer a conta
        

        if self.count == 5 or self.countReverte == 10:
            
            self.countReverte = 0
            self.reverte = 0
            self.count = 0
            self.viraEsq = 0
            self.viraDir = 0
            

        elif self.viraEsq == 1:             #se a flag para virar à esquerda ==1 viraEsquerda
            self.driveMotors((-arrayPotencias[self.count]), arrayPotencias[self.count])  
            self.count +=1

        elif self.viraDir == 1 :

            self.driveMotors(arrayPotencias[self.count], -(arrayPotencias[self.count])) #se a flag para virar à esquerda ==1 viraDireita
            self.count +=1

        elif self.reverte == 1 :
            self.driveMotors(arrayPotenciasReverte[self.countReverte], -(arrayPotenciasReverte[self.countReverte])) #se a flag para reverter ==1 
            self.countReverte +=1
        else:                          #Acabou de virar, executa o codigo normal
            

            #--------------------------DESLOCAMENTO NA HORIZONTAL--------------------------
            if(self.measures.compass >=-40 and self.measures.compass <=40) or (self.measures.compass <=-140 and self.measures.compass >=-180) or (self.measures.compass <=180 and self.measures.compass >=140):   #Se a bussola se encontra nestes graus entao o robot está na horizontal

                if (((self.measures.compass <=-160 and self.measures.compass >=-180) or (self.measures.compass <=180 and self.measures.compass >=160)) and self.orientacaoX == 1):      #se está virado para sul, mas acertou a posição em norte => PARAR MAIS TARDE
                    deslocamentoX -= 0.4
                    self.orientacaoX = 0
                    #print("CONA 1")
                elif self.measures.compass <= 30 and self.measures.compass >= -30 and self.orientacaoX == -1:  # se esta virado para norte, mas acertou a posição em sul => PARAR MAIS TARDE
                    deslocamentoX -= 0.4
                    self.orientacaoX = 0
                    #print("CONA 2")
                elif self.orientacaoX != 0:  # se apenas acertou posição, tem de parar mais cedo
                    deslocamentoX += 0.2
                    self.orientacaoX = 0
                   # print("nao mudou de orientaçao")
            
                d_x = deslocamentoX - self.previousGps[0]
                if (abs(d_x) < 2.0):  #Se a diferença entre a pos atual e anterior<2 apenas anda em frente pois não é o meio da célula
                    
                    #CÓDIGO PARA ORIENTAR O RATO NA HORIZONTAL COM A BUSSOLA PARA NAO ANDAR AOS S's (nao precisas de mexer aqui)
                    if (self.measures.compass >= -4 and self.measures.compass <0) or (self.measures.compass>= 176 and self.measures.compass<180):   #esta ligeiramente inclinado para a direita
                        self.driveMotors(0.13, 0.15)
                        l = 0.13
                        r = 0.15
                    elif (self.measures.compass <=4 and self.measures.compass > 0) or (self.measures.compass> -180 and self.measures.compass<=-176):  #esta ligeiramente inclinado para a esquerda 
                        self.driveMotors(0.15, 0.13)
                        l = 0.15
                        r = 0.13
                    else:
                        self.driveMotors(0.15, 0.15)
                        l = 0.15
                        r = 0.15


                else:   #Rato encontra-se no centro de uma nova celula (na horizontal) => retirar conclusões

                    if self.measures.compass <= 30 and self.measures.compass >= -30:    #se estiver virado para norte (direita)
                        newGPS= [newGPS[0]+2, newGPS[1]]
                    else:
                        newGPS= [newGPS[0]-2, newGPS[1]]

                    self.previousGps[0] = round(deslocamentoX)  #atualiza a posição anterior

                    print(newGPS)
                    self.driveMotors(-0.15, -0.15)                              #valores para a inércia das rodas
                    l = -0.15
                    r = -0.15
                    
                    

                    self.coordinates[newGPS[0]][newGPS[1]] = "X"  #a posicao onde se encontra esta vazia (x)
                    
                    # ---VERIFICAÇÃO DOS CENSORES---
                    # POR FLAGS NO ARRAY COM AS RESPETIVAS INFOS RETIRADAS DOS SENSORES  
                    if self.measures.compass <= 10 and self.measures.compass >= -10:    #se estiver virado para norte (direita)
                        if self.measures.irSensor[center_id] > 1.2:                     #se tiver parede à frente
                            self.coordinates[newGPS[0]+1][newGPS[1]] = "|"
                        else: 
                            self.coordinates[newGPS[0]+1][newGPS[1]] = "X"
                        if self.measures.irSensor[left_id] > 1.2:   #se tiver parede á esquerda
                            self.coordinates[newGPS[0]][newGPS[1]-1] = "-"
                        else:
                            self.coordinates[newGPS[0]][newGPS[1]-1]= "X" #se nao tiver parede quer dizer q é vazia
                        if self.measures.irSensor[right_id] > 1.2:  #se tiver parede á direita
                            self.coordinates[newGPS[0]][newGPS[1]+1] = "-"
                        else: 
                            self.coordinates[newGPS[0]][newGPS[1]+1] = "X" #se nao tiver parede quer dizer q ta vazia

                    elif abs(self.measures.compass) <= 180 and abs(self.measures.compass) >= 170: #se estiver virado para Sul (esquerda)
                        if self.measures.irSensor[center_id] > 1.2: #se tiver parede à frente
                            self.coordinates[newGPS[0]-1][newGPS[1]] = "|"
                        else: 
                            self.coordinates[newGPS[0]-1][newGPS[1]] = "X"
                        if self.measures.irSensor[left_id] > 1.2:   #se tiver parede á esquerda
                            self.coordinates[newGPS[0]][newGPS[1]+1] = "-"
                        else:
                            self.coordinates[newGPS[0]][newGPS[1]+1] = "X"
                        if self.measures.irSensor[right_id] > 1.2:  #se tiver parede á direita
                            self.coordinates[newGPS[0]][newGPS[1]-1] = "-"
                        else:  
                            self.coordinates[newGPS[0]][newGPS[1]-1] = "X"
                     
                    
                    #decide para onde vai conforme valores dos sensores
                    
                    if (self.measures.irSensor[center_id]< 1/0.72 and self.measures.compass <= 10 and self.measures.compass >= -10)\
                     or (self.measures.irSensor[center_id]< 1/0.72 and (self.measures.compass > 170 or self.measures.compass < -170)):   #Tem a possibilidade de ir em frente pois nao existe parede e não tem parede visitada
                        self.driveMotors(0.15,0.15)
                        l = 0.15
                        r = 0.15

                    elif(self.measures.irSensor[left_id]< 1/0.72 and self.measures.compass <= 10 and self.measures.compass >= -10 )\
                     or (self.measures.irSensor[left_id]< 1/0.72 and (self.measures.compass > 170 or self.measures.compass < -170)):     #Tem a possibilidade de ir para a esquerda pois á esquerda nao tem parede e nao foi visitada
                        #self.driveMotors(-0.129, 0.129)
                        self.viraEsq = 1
                        deslocamentoX = math.floor(deslocamentoX)
                        self.updatePreviousMotors(0, 0)

                    elif(self.measures.irSensor[right_id]< 1/0.72 and self.measures.compass <= 10 and self.measures.compass >= -10)\
                     or (self.measures.irSensor[right_id]< 1/0.72 and (self.measures.compass > 170 or self.measures.compass < -170)):     #Tem a possibilidade de ir para a direita pois à direita nao tem parede e nao foi visitada
                        #self.driveMotors(0.129, -0.129)
                        self.viraDir = 1
                        deslocamentoX = math.floor(deslocamentoX)
                        self.updatePreviousMotors(0, 0)

                    elif (self.measures.irSensor[right_id]>= 1/0.72 and self.measures.irSensor[center_id]>= 1/0.72 and self.measures.irSensor[left_id]>= 1/0.72):   #Está num beco, tem de inverter
                        #self.driveMotors(0.129, -0.129)
                        self.reverte = 1
                        deslocamentoX = math.floor(deslocamentoX)
                        self.updatePreviousMotors(0, 0)
                    
                    if self.measures.irSensor[center_id]> 1/0.35: # para acertar posição do robot 
                        
                        if self.measures.compass <= 10 and self.measures.compass >= -10:    #se estiver virado para norte
                            self.orientacaoX = 1
                            #print("atualizou pos 1")
                        elif (self.measures.compass <=-170 and self.measures.compass >=-180) or (self.measures.compass <=180 and self.measures.compass >=170):   #se estiver virado para sul
                            self.orientacaoX = -1
                            #print("atualizou pos 2")
                        
                   
                    # else:   #As posições à volta dele já estão todas visitadas
                        
                    #     if (self.measures.irSensor[left_id]< 1/0.72) and (self.measures.irSensor[right_id]< 1/0.72):
                            
                    #         a = random.randrange(2)
                            
                    #         if a==0:
                    #             self.driveMotors(0.129, -0.129)
                    #             self.updatePreviousMotors(0, 0)
                    #             self.viraDir = 1
                    #         elif a==1:
                    #             self.driveMotors(-0.129, 0.129)
                    #             self.updatePreviousMotors(0, 0)
                    #             self.viraEsq = 1
                    #     elif (self.measures.irSensor[left_id]< 1/0.72):
                            
                    #         self.driveMotors(-0.129, 0.129)
                    #         self.updatePreviousMotors(0, 0)
                    #         self.viraEsq = 1
                    #     elif (self.measures.irSensor[right_id]< 1/0.72):
                    #         self.driveMotors(0.129, -0.129)
                    #         self.updatePreviousMotors(0, 0)
                            
                    #         self.viraDir = 1
                    #     elif self.measures.irSensor[center_id] < 1/0.72:
                    #         self.driveMotors(0.15,0.15)
                    #         l = 0.15
                    #         r = 0.15

                # cáluco do deslocamento de cada iteração
                if(self.viraDir ==0 and self.viraEsq == 0 and self.reverte == 0):
                    
                    self.mediaCoordenadas = (((previousPowerR + previousPowerL)/2) + ((l+r)/2)) /2  #obtenção de coordenadas mais especificas
                    deslocamentoX += self.mediaCoordenadas  #determinação do deslocamento horizontal(incrementa sempre)
                    self.updatePreviousMotors(l, r) #atualiza as potencias dos motores da itereção anterior (para se usar na prox iteração e fzr a média)

                         
                        
                            

            #--------------------------DESLOCAMENTO NA VERTICAL--------------------------            
            elif (self.measures.compass >=60 and self.measures.compass <=120) or (self.measures.compass >=-120 and self.measures.compass <=-60):  #encontra-se na vertical   

                if self.measures.compass >=-120 and self.measures.compass <=-60 and self.orientacaoY == 1:      #se está virado para baixo, mas acertou a posição em cima => PARAR MAIS TARDE
                    deslocamentoY -= 0.4
                    self.orientacaoY = 0

                elif self.measures.compass >=60 and self.measures.compass <=120 and self.orientacaoY == -1:  # se esta virado para cima, mas acertou a posição em baixo => PARAR MAIS TARDE
                    deslocamentoY -= 0.4
                    self.orientacaoY = 0

                elif self.orientacaoY != 0:  # se apenas acertou posição, tem de parar mais cedo
                    deslocamentoY += 0.2
                    self.orientacaoY = 0
            
                d_y = deslocamentoY - self.previousGps[1]

                if ( abs(d_y) < 2.0):  #Se a diferença entre a pos atual e anterior<2 apenas anda em frente pois não é o meio da célula
                    
                    #CÓDIGO PARA ORIENTAR O RATO NA VERTICAL COM A BUSSOLA PARA NAO ANDAR AOS S's (nao precisas de mexer aqui)
                    if (self.measures.compass >= 86 and self.measures.compass <90) or (self.measures.compass<-90 and self.measures.compass>=-94):   #esta ligeiramente inclinado para a direita
                        self.driveMotors(0.13, 0.15)
                        l = 0.13
                        r = 0.15
                    elif (self.measures.compass <=94 and self.measures.compass > 90) or (self.measures.compass<= -86 and self.measures.compass>-90):  #esta ligeiramente inclinado para a esquerda 
                        self.driveMotors(0.15, 0.13)
                        l = 0.15
                        r = 0.13
                    else:
                        self.driveMotors(0.15, 0.15)
                        l = 0.15
                        r = 0.15
                    
                else:   #Rato encontra-se no centro de uma nova celula (na vertical) => retirar conclusões
                    
                    #código para atualizar o novo gps
                    if self.measures.compass >=60 and self.measures.compass <=120:  #Se estiver virado para cima
                        newGPS= [newGPS[0], newGPS[1]+2]
                    else:                                                           #Se estiver virado para baixo
                        newGPS= [newGPS[0], newGPS[1]-2]

                    print(newGPS)
                    self.previousGps[1] = round(deslocamentoY) #atualiza a posição anterior

                    self.driveMotors(-0.15, -0.15)
                    l = -0.15
                    r = -0.15
                    self.coordinates[newGPS[0]][newGPS[1]] = "X" #a posicao onde se encontra esta vazia (x)
                   
                    # ---VERIFICAÇÃO DOS CENSORES---
                    # POR FLAGS NO ARRAY COM AS RESPETIVAS INFOS RETIRADAS DOS SENSORES
                    if self.measures.compass <= 100 and self.measures.compass >= 80: #se estiver virado para cima
                        if self.measures.irSensor[center_id] > 1.2: #se tiver parede à frente
                            self.coordinates[newGPS[0]][newGPS[1]-1] = "-"
                        else: 
                            self.coordinates[newGPS[0]][newGPS[1]-1] = "X"
                        if self.measures.irSensor[left_id] > 1.2: #se tiver parede á esquerda
                            self.coordinates[newGPS[0]-1][newGPS[1]] = "|"
                        else:
                            self.coordinates[newGPS[0]-1][newGPS[1]] = "X" 
                        if self.measures.irSensor[right_id] > 1.2: #se tiver parede á direita not sure
                            self.coordinates[newGPS[0]+1][newGPS[1]] = "|"
                        else:
                            self.coordinates[newGPS[0]+1][newGPS[1]] = "X"

                    elif abs(self.measures.compass) <= 100 and abs(self.measures.compass) >= 80: #se estiver virado para baixo
                        if self.measures.irSensor[center_id] > 1.2: #se tiver parede à frente
                            self.coordinates[newGPS[0]][newGPS[1]+1] = "-"
                        else: 
                            self.coordinates[newGPS[0]][newGPS[1]+1] = "X"
                        if self.measures.irSensor[left_id] > 1.2: #se tiver parede á esquerda
                            self.coordinates[newGPS[0]+1][newGPS[1]] = "|"
                        else:
                            self.coordinates[newGPS[0]+1][newGPS[1]] = "X"
                        if self.measures.irSensor[right_id] > 1.2: #se tiver parede á direita
                            self.coordinates[newGPS[0]-1][newGPS[1]] = "|"
                        else:
                            self.coordinates[newGPS[0]-1][newGPS[1]] = "X"

                    #decide para onde vai conforme valores dos sensores
                    
                    if (self.measures.irSensor[center_id]< 1/0.72 and self.measures.compass <= 100 and self.measures.compass >= 80)\
                     or (self.measures.irSensor[center_id]< 1/0.72 and (self.measures.compass > -100 and self.measures.compass < -80)):   #Tem a possibilidade de ir em frente pois nao existe parede e não foi visitada
                        self.driveMotors(0.15,0.15)
                        l = 0.15
                        r = 0.15

                    elif(self.measures.irSensor[left_id]< 1/0.72 and self.measures.compass <= 100 and self.measures.compass >= 80)\
                     or (self.measures.irSensor[left_id]< 1/0.72 and (self.measures.compass > -100 and self.measures.compass < -80)):     #Tem a possibilidade de ir para a esquerda pois á esquerda nao tem parede e nao foi visitada

                        self.viraEsq = 1
                        deslocamentoY = math.floor(deslocamentoY)

                    elif(self.measures.irSensor[right_id]< 1/0.72 and self.measures.compass <= 100 and self.measures.compass >= 80)\
                     or (self.measures.irSensor[right_id]< 1/0.72 and (self.measures.compass > -100 and self.measures.compass < -80)):     #Tem a possibilidade de ir para a direita pois à direita nao tem parede e nao foi visitada

                        self.viraDir = 1
                        deslocamentoY = math.floor(deslocamentoY)
                        self.updatePreviousMotors(0, 0)

                    elif (self.measures.irSensor[right_id]>= 1/0.72 and self.measures.irSensor[center_id]>= 1/0.72 and self.measures.irSensor[left_id]>= 1/0.72):   #Está num beco, tem de inverter

                        self.reverte = 1
                        deslocamentoY = math.floor(deslocamentoY)
                        self.updatePreviousMotors(0, 0)
                    
                    if self.measures.irSensor[center_id]> 1/0.35: # para acertar posição do robot 
                        
                        if self.measures.compass >=60 and self.measures.compass <=120:    #se estiver virado para cima
                            self.orientacaoY = 1
                            #print("atualizou pos 1")
                        elif self.measures.compass >=-120 and self.measures.compass <=-60:   #se estiver virado para baixo
                            self.orientacaoY = -1
                            #print("atualizou pos 2")

                    # else:   #As posições à volta dele já estão todas visitadas
                       
                    #     if (self.measures.irSensor[left_id]< 1/0.72) and (self.measures.irSensor[right_id]< 1/0.72):
                    #         a = random.randrange(2)
                           
                    #         if a==0:
                    #             self.driveMotors(0.129, -0.129)
                    #             self.updatePreviousMotors(0, 0)
                    #             self.viraDir = 1
                    #         elif a==1:
                    #             self.driveMotors(-0.129, 0.129)
                    #             self.updatePreviousMotors(0, 0)
                    #             self.viraEsq = 1
                    #     elif self.measures.irSensor[center_id] < 1/0.72:
                    #         self.driveMotors(0.15,0.15)
                    #         l = 0.15
                    #         r = 0.15
                    #     elif (self.measures.irSensor[left_id]< 1/0.72):
                    #         self.driveMotors(-0.129, 0.129)
                    #         self.updatePreviousMotors(0, 0)
                    #         self.viraEsq = 1
                    #     elif (self.measures.irSensor[right_id]< 1/0.72):
                    #         self.driveMotors(0.129, -0.129)
                    #         self.updatePreviousMotors(0, 0)
                    #         self.viraDir = 1

                # cáluco do deslocamento de cada iteração
                if(self.viraDir ==0 and self.viraEsq == 0 and self.reverte == 0):
                    
                    self.mediaCoordenadas = (((previousPowerR + previousPowerL)/2) + ((l+r)/2)) /2  #obtenção de coordenadas mais especificas
                    deslocamentoY += self.mediaCoordenadas  #determinação do deslocamento horizontal(incrementa sempre)
                    self.updatePreviousMotors(l, r) #atualiza as potencias dos motores da itereção anterior (para se usar na prox iteração e fzr a média)
    

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
pos = 0
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