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
import a_star_path_finding as pf

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
        self.andaComAstar = 0
        
        self.origin = (0,0)
        self.visitableCoord = (0,0) #coordenadas que vao ser guardadas no array visitable
        self.visitable = [] #array onde ficam guardadas as posicoes que foram marcadas como visitaveis, mas nao chegaram a ser visitadas
        self.visitableNoRep = [] ##array de visitaveis nde nao ha posicoes repetidades
        self.visited = [] #array com as posicoes ja visitadas

        self.previousGps = [0,0]
        self.contadorCiclos = 0

        self.foo = " " #inicializa todos os espaços com " "
        self.coordinates = [[self.foo for x in range(55)] for y in range(27)] #cria um array bidimensional [56][27]
        self.coordinates[13][27] = "I" #posicao inicial
        self.walls = []
        for x in range(57):
            for y in range(28):
                self.walls.append((x,y))
        
        self.min = 12345
        self.destinoCurto = (0,0)
        #self.path = [[0,0]] 
        self.smallestPath = [] #caminho mais curto
        
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
        
    def verifySensorsXX(self):  #função para leitura dos sensores na horizontal

        center_id = 0
        left_id = 1
        right_id = 2

        if newGPS[0]== 27 and newGPS[1] == 13:
            self.coordinates[newGPS[1]][newGPS[0]] = "I"  #a posicao onde se encontra é o inicio (I)
        else:
            self.coordinates[newGPS[1]][newGPS[0]] = "X" #a posicao onde se encontra esta vazia (x)
        try:
            self.walls.remove((newGPS[0], newGPS[1]))
        except:
            pass
        
        # ---VERIFICAÇÃO DOS CENSORES---
        # POR FLAGS NO ARRAY COM AS RESPETIVAS INFOS RETIRADAS DOS SENSORES  
        if self.measures.compass <= 10 and self.measures.compass >= -10:    #se estiver virado para norte (direita)
            if self.measures.irSensor[center_id] > 1.2:                     #se tiver parede à frente
                self.coordinates[newGPS[1]][newGPS[0]+1] = "|"
            else: 
                self.coordinates[newGPS[1]][newGPS[0]+1] = "X"
                self.coordinates[newGPS[1]][newGPS[0]+2] = "X"
                try:
                    self.walls.remove((newGPS[0]+1, newGPS[1]))
                    self.walls.remove((newGPS[0]+2, newGPS[1]))
                except:
                    pass
            if self.measures.irSensor[left_id] > 1.2:   #se tiver parede á esquerda
                self.coordinates[newGPS[1]-1][newGPS[0]] = "-"
            else:
                self.coordinates[newGPS[1]-1][newGPS[0]] = "X" #se nao tiver parede quer dizer q é vazia
                self.coordinates[newGPS[1]-2][newGPS[0]] = "X" #se nao tiver parede quer dizer q é vazia
                if not (self.measures.irSensor[center_id] > 1.2):#se nao tiver parede em frente nem a esquerda, a casa e adicionada as visitaveis
                    self.visitableCoord = (newGPS[0], newGPS[1]-2) 
                    self.visitable.append(self.visitableCoord) #essa casa e acrescentada ao array de casas que podem ser visitadas

                try:
                    self.walls.remove((newGPS[0], newGPS[1]-1))
                    self.walls.remove((newGPS[0], newGPS[1]-2))
                except:
                    pass
            if self.measures.irSensor[right_id] > 1.2:  #se tiver parede á direita
                self.coordinates[newGPS[1]+1][newGPS[0]] = "-"
            else: 
                self.coordinates[newGPS[1]+1][newGPS[0]] = "X" #se nao tiver parede quer dizer q ta vazia
                self.coordinates[newGPS[1]+2][newGPS[0]] = "X" #se nao tiver parede quer dizer q ta vazia

                if not (self.measures.irSensor[center_id] > 1.2): #se nao tiver parede a frente nem a direita vai em frente
                    self.visitableCoord = (newGPS[0], newGPS[1]+2)
                    self.visitable.append(self.visitableCoord) #essa casa e acrescentada ao array de casas que podem ser visitadas
                elif not (self.measures.irSensor[left_id] > 1.2): #se nao tiver parede a esquerda nem a direita, vai para a esquerda logo
                    self.visitableCoord = (newGPS[0], newGPS[1]+2) 
                    self.visitable.append(self.visitableCoord) #essa casa e acrescentada ao array de casas que podem ser visitadas

                try:
                    self.walls.remove((newGPS[0], newGPS[1]+1))
                    self.walls.remove((newGPS[0], newGPS[1]+2))
                except:
                    pass
        elif abs(self.measures.compass) <= 180 and abs(self.measures.compass) >= 170: #se estiver virado para Sul (esquerda)
            if self.measures.irSensor[center_id] > 1.2: #se tiver parede à frente
                self.coordinates[newGPS[1]][newGPS[0]-1] = "|"
            else: 
                self.coordinates[newGPS[1]][newGPS[0]-1] = "X"
                self.coordinates[newGPS[1]][newGPS[0]-2] = "X"
                try:
                    self.walls.remove((newGPS[0]-1, newGPS[1]))
                    self.walls.remove((newGPS[0]-2, newGPS[1]))
                except:
                    pass
            if self.measures.irSensor[left_id] > 1.2:   #se tiver parede á esquerda
                self.coordinates[newGPS[1]+1][newGPS[0]] = "-"
            else:
                self.coordinates[newGPS[1]+1][newGPS[0]] = "X"
                self.coordinates[newGPS[1]+2][newGPS[0]] = "X"
                if not (self.measures.irSensor[center_id] > 1.2):#se nao tiver parede em frente nem a esquerda, a casa e adicionada as visitaveis
                    self.visitableCoord = (newGPS[0], newGPS[1]+2) 
                    self.visitable.append(self.visitableCoord) #essa casa e acrescentada ao array de casas que podem ser visitadas

                try:
                    self.walls.remove((newGPS[0], newGPS[1]+1))
                    self.walls.remove((newGPS[0], newGPS[1]+2))
                except:
                    pass
            if self.measures.irSensor[right_id] > 1.2:  #se tiver parede á direita
                self.coordinates[newGPS[1]-1][newGPS[0]] = "-"
            else:  
                self.coordinates[newGPS[1]-1][newGPS[0]] = "X"
                self.coordinates[newGPS[1]-2][newGPS[0]] = "X"
                if not (self.measures.irSensor[center_id] > 1.2): #se nao tiver parede a frente nem a direita vai em frente
                    self.visitableCoord = (newGPS[0], newGPS[1]-2) 
                    self.visitable.append(self.visitableCoord) #essa casa e acrescentada ao array de casas que podem ser visitadas
                elif not (self.measures.irSensor[left_id] > 1.2): #se nao tiver parede a esquerda nem a direita, vai para a esquerda logo
                    self.visitableCoord = (newGPS[0], newGPS[1]-2) 
                    self.visitable.append(self.visitableCoord) #essa casa e acrescentada ao array de casas que podem ser visitadas

                try:
                    self.walls.remove((newGPS[0], newGPS[1]-1))
                    self.walls.remove((newGPS[0], newGPS[1]-2))
                except:
                    pass


    def verifySensorsYY(self):  #função para leitura dos sensores na vertical
        center_id = 0
        left_id = 1
        right_id = 2

        if newGPS[0]== 27 and newGPS[1] == 13:
            self.coordinates[newGPS[1]][newGPS[0]] = "I"  #a posicao onde se encontra é o inicio (I)
        else:
            self.coordinates[newGPS[1]][newGPS[0]] = "X" #a posicao onde se encontra esta vazia (x)
        try:
            self.walls.remove((newGPS[0], newGPS[1]))
        except:
            pass
        # ---VERIFICAÇÃO DOS CENSORES---
        # POR FLAGS NO ARRAY COM AS RESPETIVAS INFOS RETIRADAS DOS SENSORES
        if self.measures.compass <= 100 and self.measures.compass >= 80: #se estiver virado para cima
            if self.measures.irSensor[center_id] > 1.2: #se tiver parede à frente
                self.coordinates[newGPS[1]-1][newGPS[0]] = "-"
            else: 
                self.coordinates[newGPS[1]-1][newGPS[0]] = "X"
                self.coordinates[newGPS[1]-2][newGPS[0]] = "X"
                try:
                    self.walls.remove((newGPS[0], newGPS[1]-1))
                    self.walls.remove((newGPS[0], newGPS[1]-2))
                except:
                    pass
            if self.measures.irSensor[left_id] > 1.2: #se tiver parede á esquerda
                self.coordinates[newGPS[1]][newGPS[0]-1] = "|"
            else:
                self.coordinates[newGPS[1]][newGPS[0]-1] = "X" 
                self.coordinates[newGPS[1]][newGPS[0]-2] = "X"
                if not (self.measures.irSensor[center_id] > 1.2):#se nao tiver parede em frente nem a esquerda, a casa e adicionada as visitaveis
                    self.visitableCoord = (newGPS[0]-2, newGPS[1]) 
                    self.visitable.append(self.visitableCoord) #essa casa e acrescentada ao array de casas que podem ser visitadas

                try:
                    self.walls.remove((newGPS[0]-1, newGPS[1]))
                    self.walls.remove((newGPS[0]-2, newGPS[1]))
                except:
                    pass
            if self.measures.irSensor[right_id] > 1.2: #se tiver parede á direita not sure
                self.coordinates[newGPS[1]][newGPS[0]+1] = "|"
            else:
                self.coordinates[newGPS[1]][newGPS[0]+1] = "X"
                self.coordinates[newGPS[1]][newGPS[0]+2] = "X"
                if not (self.measures.irSensor[center_id] > 1.2): #se nao tiver parede a frente nem a direita vai em frente
                    self.visitableCoord = (newGPS[0]+2, newGPS[1]) 
                    self.visitable.append(self.visitableCoord) #essa casa e acrescentada ao array de casas que podem ser visitadas
                elif not (self.measures.irSensor[left_id] > 1.2): #se nao tiver parede a esquerda nem a direita, vai para a esquerda logo
                    self.visitableCoord = (newGPS[0]+2, newGPS[1]) 
                    self.visitable.append(self.visitableCoord) #essa casa e acrescentada ao array de casas que podem ser visitadas

                try:
                    self.walls.remove((newGPS[0]+1, newGPS[1]))
                    self.walls.remove((newGPS[0]+2, newGPS[1]))
                except:
                    pass

        elif abs(self.measures.compass) <= 100 and abs(self.measures.compass) >= 80: #se estiver virado para baixo
            if self.measures.irSensor[center_id] > 1.2: #se tiver parede à frente
                self.coordinates[newGPS[1]+1][newGPS[0]] = "-"
            else: 
                self.coordinates[newGPS[1]+1][newGPS[0]] = "X"
                self.coordinates[newGPS[1]+2][newGPS[0]] = "X"
                try:
                    self.walls.remove((newGPS[0], newGPS[1]+1))
                    self.walls.remove((newGPS[0], newGPS[1]+2))
                except:
                    pass
            if self.measures.irSensor[left_id] > 1.2: #se tiver parede á esquerda
                self.coordinates[newGPS[1]][newGPS[0]+1] = "|"
            else:
                self.coordinates[newGPS[1]][newGPS[0]+1] = "X"
                self.coordinates[newGPS[1]][newGPS[0]+2] = "X"
                if not (self.measures.irSensor[center_id] > 1.2):#se nao tiver parede em frente nem a esquerda, a casa e adicionada as visitaveis
                    self.visitableCoord = (newGPS[0]+2, newGPS[1]) 
                    self.visitable.append(self.visitableCoord) #essa casa e acrescentada ao array de casas que podem ser visitadas

                try:
                    self.walls.remove((newGPS[0]+1, newGPS[1]))
                    self.walls.remove((newGPS[0]+2, newGPS[1]))
                except:
                    pass
            if self.measures.irSensor[right_id] > 1.2: #se tiver parede á direita
                self.coordinates[newGPS[1]][newGPS[0]-1] = "|"
            else:
                self.coordinates[newGPS[1]][newGPS[0]-1] = "X"
                self.coordinates[newGPS[1]][newGPS[0]-2] = "X"
                if not (self.measures.irSensor[center_id] > 1.2): #se nao tiver parede a frente nem a direita vai em frente
                    self.visitableCoord = (newGPS[0]-2, newGPS[1])
                    self.visitable.append(self.visitableCoord) #essa casa e acrescentada ao array de casas que podem ser visitadas
                elif not (self.measures.irSensor[left_id] > 1.2): #se nao tiver parede a esquerda nem a direita, vai para a esquerda logo
                    self.visitableCoord = (newGPS[0]-2, newGPS[1]) 
                    self.visitable.append(self.visitableCoord) #essa casa e acrescentada ao array de casas que podem ser visitadas

                try:
                    self.walls.remove((newGPS[0]-1, newGPS[1]))
                    self.walls.remove((newGPS[0]-2, newGPS[1]))
                except:
                    pass

    def updateSmallestPath(self):

        if len(self.smallestPath) == 1: #se so tiver uma posição no path
            self.smallestPath.pop(0)
        else:                           #se tiver varias posições
            self.smallestPath.pop(0)
            self.smallestPath.pop(0)

        if len(self.smallestPath) == 0:#se já executou todos os steps, termina de andar com o A*
            self.andaComAstar = 0   #Para de andar com o A*

    def wander(self):
        
        global deslocamentoX
        global deslocamentoY
        global arrayPotencias
        global newGPS
        global arrayPotenciasReverte

        center_id = 0
        left_id = 1
        right_id = 2



        if self.contadorCiclos == 0: #PRIMEIRO CICLO
            self.updatePreviousMotors(0, 0) #no 1º ciclo a previousMotors é zero
            deslocamentoX = 27
            deslocamentoY = 13

            self.verifySensorsXX()  #só chamamos esta função, pois ele começa sempre na horizontal
            self.contadorCiclos+=1
            self.previousGps = [27, 13]


        #---------------------------ESCREVER NO FICHEIRO-----------------------------------------------
        with open('map.out', 'w') as outfile: 
            for i in range(27):
                for j in range(55):
                    outfile.write(self.coordinates[i][j])
                    #print(self.coordinates[i][j])
                outfile.write("\n")
        print("\n".join(["".join([x for x in row])for row in self.coordinates]))


        #-------------------------------------ROTAÇÕES---------------------------------------------------------
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
        else:                    #Acabou de virar, executa o codigo normal
            

        #-------------------------------DESLOCAMENTO NA HORIZONTAL-------------------------------

            if(self.measures.compass >=-40 and self.measures.compass <=40) or (self.measures.compass <=-140 and self.measures.compass >=-180) or (self.measures.compass <=180 and self.measures.compass >=140):   #Se a bussola se encontra nestes graus entao o robot está na horizontal

                if (((self.measures.compass <=-160 and self.measures.compass >=-180) or (self.measures.compass <=180 and self.measures.compass >=160)) and self.orientacaoX == 1):      #se está virado para sul, mas acertou a posição em norte => PARAR MAIS TARDE
                    deslocamentoX -= 0.395
                    self.orientacaoX = 0
                elif self.measures.compass <= 30 and self.measures.compass >= -30 and self.orientacaoX == -1:  # se esta virado para norte, mas acertou a posição em sul => PARAR MAIS TARDE
                    deslocamentoX -= 0.395
                    self.orientacaoX = 0
                elif self.orientacaoX != 0:  # se apenas acertou posição, tem de parar mais cedo
                    deslocamentoX += 0.2
                    self.orientacaoX = 0
            
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
                    self.contadorCiclos +=1

                    if self.measures.compass <= 30 and self.measures.compass >= -30:    #se estiver virado para norte (direita)
                        newGPS= [newGPS[0]+2, newGPS[1]]
                    else:
                        newGPS= [newGPS[0]-2, newGPS[1]]

                    self.previousGps[0] = round(deslocamentoX)  #atualiza a posição anterior

                    #print(newGPS)

                    self.driveMotors(-0.15, -0.15)                              #valores para a inércia das rodas
                    l = -0.15
                    r = -0.15
                    
                    if (newGPS[0], newGPS[1]) not in self.visited: #se essa casa ainda nao tiver registada como visitada, e acrescentada as casas visitadas
                        self.visited.append((newGPS[0], newGPS[1]))
                        
                    self.verifySensorsXX()

                    for item in self.visitable: #remove as casas visitaveis repetidas
                        if item not in self.visitableNoRep and (item not in self.visited): 
                            self.visitableNoRep.append(item) #e guarda neste array
                    
                   # print("visitable: ", self.visitableNoRep)
                   # print("visited: ", self.visited)

                    #decide para onde vai conforme valores dos sensores
                    
                    if ((self.measures.irSensor[center_id]< 1/0.72 and self.measures.compass <= 10 and self.measures.compass >= -10 and (newGPS[0]+2,newGPS[1]) not in self.visited )\
                     or (self.measures.irSensor[center_id]< 1/0.72 and (self.measures.compass > 170 or self.measures.compass < -170) and (newGPS[0]-2,newGPS[1]) not in self.visited)) and self.andaComAstar == 0:   #Tem a possibilidade de ir em frente pois nao existe parede e não tem parede visitada
                        self.driveMotors(0.15,0.15)
                        l = 0.15
                        r = 0.15

                    elif((self.measures.irSensor[left_id]< 1/0.72 and self.measures.compass <= 10 and self.measures.compass >= -10 and (newGPS[0],newGPS[1]-2) not in self.visited)\
                     or (self.measures.irSensor[left_id]< 1/0.72 and (self.measures.compass > 170 or self.measures.compass < -170) and (newGPS[0],newGPS[1]+2) not in self.visited)) and self.andaComAstar == 0:     #Tem a possibilidade de ir para a esquerda pois á esquerda nao tem parede e nao foi visitada
                        #self.driveMotors(-0.129, 0.129)
                        self.viraEsq = 1
                        deslocamentoX = math.floor(deslocamentoX)
                        self.updatePreviousMotors(0, 0)

                    elif((self.measures.irSensor[right_id]< 1/0.72 and self.measures.compass <= 10 and self.measures.compass >= -10 and (newGPS[0],newGPS[1]+2) not in self.visited)\
                     or (self.measures.irSensor[right_id]< 1/0.72 and (self.measures.compass > 170 or self.measures.compass < -170) and (newGPS[0],newGPS[1]-2) not in self.visited)) and self.andaComAstar == 0:     #Tem a possibilidade de ir para a direita pois à direita nao tem parede e nao foi visitada
                        #self.driveMotors(0.129, -0.129)
                        self.viraDir = 1
                        deslocamentoX = math.floor(deslocamentoX)
                        self.updatePreviousMotors(0, 0)

                    elif ((self.measures.irSensor[right_id]>= 1/0.72 and self.measures.irSensor[center_id]>= 1/0.72 and self.measures.irSensor[left_id]>= 1/0.72)) and self.andaComAstar == 0:   #Está num beco, tem de inverter
                        #self.driveMotors(0.129, -0.129)
                        self.reverte = 1
                        deslocamentoX = math.floor(deslocamentoX)
                        self.updatePreviousMotors(0, 0)
                    
                    else:   #As posições à volta dele já estão todas visitadas

                        if self.andaComAstar == 0:#se ainda nao estiver a andar com o A*, ainda nao precisa de executar o codigo deste if

                            a = pf.AStar()
                            self.origin = (newGPS[0], newGPS[1])
                            
                            for destiny in self.visitableNoRep:
                                x1 = destiny[0]
                                y1 = destiny[1]
                                x2 = newGPS[0]
                                y2 = newGPS[1]
                                if (x1 == x2 and y1 == y2): #se destino igual ao inicio, já foi visitad, logo nao vale a pena ir!
                                    self.visitableNoRep.remove((x1, y1))
                                else:
                                    dist = sqrt( (x2 - x1)**2 + (y2 - y1)**2 )
                                    if dist < self.min:
                                        self.min = dist
                                        self.destinoCurto = ((x1, y1))
     
                            a.init_grid(56, 27, self.walls, self.origin, self.destinoCurto)
                            self.smallestPath = a.solve()
                            self.min = 1234

                            for vis in range(len(self.smallestPath)): #para cada casa no smallestPath
                                if self.smallestPath[vis] in self.visitableNoRep: #se existir no array de visitaveis
                                    self.visitableNoRep.remove(self.smallestPath[vis]) #e retirado porque ja vai ser visitada

                            self.andaComAstar = 1
                            self.smallestPath.pop(0) #retira o ponto de partida
                            self.smallestPath.pop(0) #retira tambem a seguinte

                        if len(self.smallestPath) != 0:
                            primeiraCoordenada = self.smallestPath[0]
                            joystick = (newGPS[0] -primeiraCoordenada[0], newGPS[1] -primeiraCoordenada[1])
                            
                            if self.measures.compass <= 30 and self.measures.compass >= -30: #se virado para a direita
                                if(joystick == (-2,0)):
                                    self.driveMotors(0.15,0.15)
                                    l = 0.15
                                    r = 0.15
                                    self.updateSmallestPath()

                                elif(joystick == (2,0)):
                                    self.reverte = 1
                                    deslocamentoX = math.floor(deslocamentoX)
                                    self.updatePreviousMotors(0, 0)
                                    self.updateSmallestPath()

                                elif (joystick == (0,2)):
                                    print("vira esquerda E vai em frente")
                                    self.viraEsq = 1
                                    deslocamentoX = math.floor(deslocamentoX)
                                    self.updatePreviousMotors(0, 0)
                                    self.updateSmallestPath()

                                elif (joystick == (0,-2)):
                                    print("vira direita E vai em frente")
                                    self.viraDir = 1
                                    deslocamentoX = math.floor(deslocamentoX)
                                    self.updatePreviousMotors(0, 0)
                                    self.updateSmallestPath()

                            else: #se esta virado para a esquerda

                                if(joystick == (2,0) ):
                                    print("vai em frente")
                                    self.driveMotors(0.15,0.15)
                                    l = 0.15
                                    r = 0.15
                                    self.updateSmallestPath()

                                elif(joystick == (-2,0)):
                                    print("Reverte E vai em frente")
                                    self.reverte = 1
                                    deslocamentoX = math.floor(deslocamentoX)
                                    self.updatePreviousMotors(0, 0)
                                    self.updateSmallestPath()

                                elif (joystick == (0,2)):
                                    print("vira esquerda se virado para a direita E vai em frente")
                                    self.viraDir = 1
                                    deslocamentoX = math.floor(deslocamentoX)
                                    self.updatePreviousMotors(0, 0)
                                    self.updateSmallestPath()

                                elif (joystick == (0,-2)):
                                    print("vira direita se virado para a direita E vai em frente")
                                    self.viraEsq = 1
                                    deslocamentoX = math.floor(deslocamentoX)
                                    self.updatePreviousMotors(0, 0)
                                    self.updateSmallestPath()


                    # para acertar posição do robot 
                    if self.measures.irSensor[center_id]> 1/0.35: 
                        
                        if self.measures.compass <= 10 and self.measures.compass >= -10:    #se estiver virado para norte
                            self.orientacaoX = 1
                        elif (self.measures.compass <=-170 and self.measures.compass >=-180) or (self.measures.compass <=180 and self.measures.compass >=170):   #se estiver virado para sul
                            self.orientacaoX = -1
                        

                # cáluco do deslocamento de cada iteração
                if(self.viraDir ==0 and self.viraEsq == 0 and self.reverte == 0):
                    
                    self.mediaCoordenadas = (((previousPowerR + previousPowerL)/2) + ((l+r)/2)) /2  #obtenção de coordenadas mais especificas
                    deslocamentoX += self.mediaCoordenadas  #determinação do deslocamento horizontal(incrementa sempre)
                    self.updatePreviousMotors(l, r) #atualiza as potencias dos motores da itereção anterior (para se usar na prox iteração e fzr a média)

                         
                        
                            

            #--------------------------DESLOCAMENTO NA VERTICAL--------------------------            
            elif (self.measures.compass >=60 and self.measures.compass <=120) or (self.measures.compass >=-120 and self.measures.compass <=-60):  #encontra-se na vertical   

                if self.measures.compass >=-120 and self.measures.compass <=-60 and self.orientacaoY == 1:      #se está virado para baixo, mas acertou a posição em cima => PARAR MAIS TARDE
                    deslocamentoY -= 0.395
                    self.orientacaoY = 0

                elif self.measures.compass >=60 and self.measures.compass <=120 and self.orientacaoY == -1:  # se esta virado para cima, mas acertou a posição em baixo => PARAR MAIS TARDE
                    deslocamentoY -= 0.395
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
                    self.contadorCiclos +=1
                    #código para atualizar o novo gps
                    if self.measures.compass >=60 and self.measures.compass <=120:  #Se estiver virado para cima
                        newGPS= [newGPS[0], newGPS[1]-2]
                    else:                                                           #Se estiver virado para baixo
                        newGPS= [newGPS[0], newGPS[1]+2]

                    self.previousGps[1] = round(deslocamentoY) #atualiza a posição anterior

                    self.driveMotors(-0.15, -0.15)
                    l = -0.15
                    r = -0.15
                    
                    if (newGPS[0], newGPS[1]) not in self.visited: #se essa casa ainda nao tiver registada como visitada, e acrescentada as casas visitadas
                        self.visited.append((newGPS[0], newGPS[1]))
                    
                    self.verifySensorsYY()
                    
                    for item in self.visitable:
                        if item not in self.visitableNoRep and (item not in self.visited):
                            self.visitableNoRep.append(item)

                    #decide para onde vai conforme valores dos sensores
                    
                    if ((self.measures.irSensor[center_id]< 1/0.72 and self.measures.compass <= 100 and self.measures.compass >= 80 and (newGPS[0],newGPS[1]-2) not in self.visited)\
                     or (self.measures.irSensor[center_id]< 1/0.72 and (self.measures.compass > -100 and self.measures.compass < -80) and (newGPS[0],newGPS[1]+2) not in self.visited)) and self.andaComAstar == 0:   #Tem a possibilidade de ir em frente pois nao existe parede e não foi visitada
                        self.driveMotors(0.15,0.15)
                        l = 0.15
                        r = 0.15

                    elif((self.measures.irSensor[left_id]< 1/0.72 and self.measures.compass <= 100 and self.measures.compass >= 80 and (newGPS[0]-2,newGPS[1]) not in self.visited)\
                     or (self.measures.irSensor[left_id]< 1/0.72 and (self.measures.compass > -100 and self.measures.compass < -80) and (newGPS[0]+2,newGPS[1]) not in self.visited)) and self.andaComAstar == 0:     #Tem a possibilidade de ir para a esquerda pois á esquerda nao tem parede e nao foi visitada                       
                        self.viraEsq = 1
                        deslocamentoY = math.floor(deslocamentoY)
                        self.updatePreviousMotors(0, 0)

                    elif((self.measures.irSensor[right_id]< 1/0.72 and self.measures.compass <= 100 and self.measures.compass >= 80 and (newGPS[0]+2,newGPS[1]) not in self.visited)\
                     or (self.measures.irSensor[right_id]< 1/0.72 and (self.measures.compass > -100 and self.measures.compass < -80) and (newGPS[0]-2,newGPS[1]) not in self.visited)) and self.andaComAstar == 0:     #Tem a possibilidade de ir para a direita pois à direita nao tem parede e nao foi visitada                        
                        self.viraDir = 1
                        deslocamentoY = math.floor(deslocamentoY)
                        self.updatePreviousMotors(0, 0)

                    elif ((self.measures.irSensor[right_id]>= 1/0.72 and self.measures.irSensor[center_id]>= 1/0.72 and self.measures.irSensor[left_id]>= 1/0.72)) and self.andaComAstar == 0:   #Está num beco, tem de inverter
                        self.reverte = 1
                        deslocamentoY = math.floor(deslocamentoY)
                        self.updatePreviousMotors(0, 0)
                    
                    else:   #As posições à volta dele já estão todas visitadas e ja nao ha x's por preencher
                        if self.andaComAstar == 0:  #se ainda nao estiver a andar com o A*, ainda nao precisa de executar o codigo deste if
                            a = pf.AStar()
                            self.origin = (newGPS[0], newGPS[1])
                            
                            for destiny in self.visitableNoRep: #para ver qual a visitavel mais perto
                                x1 = destiny[0]
                                y1 = destiny[1]
                                x2 = newGPS[0]
                                y2 = newGPS[1]

                                if (x1 == x2 and y1 == y2):
                                    self.visitableNoRep.remove((x1, y1))
                                else:
                                    dist = sqrt( (x2 - x1)**2 + (y2 - y1)**2 )
                                    if dist < self.min:
                                        self.min = dist
                                        self.destinoCurto = (x1, y1)
                            a.init_grid(56, 27, self.walls, self.origin, self.destinoCurto)
                            self.smallestPath = a.solve()
                            self.min = 1234 #volta a por um minimo muito grande para na proxima iteração achar novo minimo

                            for vis in range(len(self.smallestPath)): #para cada casa no smallestPath
                                if self.smallestPath[vis] in self.visitableNoRep: #se existir no array de visitaveis
                                    self.visitableNoRep.remove(self.smallestPath[vis]) #e retirado porque ja vai ser visitada

                            self.andaComAstar = 1
                            self.smallestPath.pop(0) #retira o ponto de partida
                            self.smallestPath.pop(0) #retira tambem a seguinte

                        if len(self.smallestPath) != 0: #enquanto houver smallest path, anda com o joystick
                            primeiraCoordenada = self.smallestPath[0]
                            joystick = (newGPS[0] -primeiraCoordenada[0], newGPS[1] -primeiraCoordenada[1])

                            if self.measures.compass <= 120 and self.measures.compass >= 60: #se virado para cima
                                if(joystick == (0,2)):
                                    self.driveMotors(0.15,0.15)
                                    l = 0.15
                                    r = 0.15
                                    self.updateSmallestPath()

                                elif(joystick == (0,-2)):
                                    self.reverte = 1
                                    deslocamentoY = math.floor(deslocamentoY)
                                    self.updatePreviousMotors(0, 0)
                                    self.updateSmallestPath()

                                elif (joystick == (2,0)):
                                    self.viraEsq = 1
                                    self.updateSmallestPath()
                                    deslocamentoY = math.floor(deslocamentoY)
                                    self.updatePreviousMotors(0, 0)

                                elif (joystick == (-2,0)):
                                    self.viraDir = 1                                   
                                    deslocamentoY = math.floor(deslocamentoY)
                                    self.updatePreviousMotors(0, 0)
                                    self.updateSmallestPath()

                            else: #se esta virado para baixo

                                if(joystick == (0,-2)):
                                    self.driveMotors(0.15,0.15)
                                    l = 0.15
                                    r = 0.15
                                    self.updateSmallestPath()

                                elif(joystick == (0,2)):
                                    self.reverte = 1
                                    deslocamentoY = math.floor(deslocamentoY)
                                    self.updatePreviousMotors(0, 0)
                                    self.updateSmallestPath()

                                elif (joystick == (-2,0)):
                                    self.viraEsq = 1
                                    deslocamentoY = math.floor(deslocamentoY)
                                    self.updatePreviousMotors(0, 0)
                                    self.updateSmallestPath()

                                elif (joystick == (2,0)):
                                    self.viraDir = 1
                                    deslocamentoY = math.floor(deslocamentoY)
                                    self.updatePreviousMotors(0, 0)
                                    self.updateSmallestPath()

                    # para acertar posição do robot 
                    if self.measures.irSensor[center_id]> 1/0.35: #se estiver muito perto de uma parede corrige pos
                        if self.measures.compass >=60 and self.measures.compass <=120:    #se estiver virado para cima
                            self.orientacaoY = 1
                        elif self.measures.compass >=-120 and self.measures.compass <=-60:   #se estiver virado para baixo
                            self.orientacaoY = -1

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