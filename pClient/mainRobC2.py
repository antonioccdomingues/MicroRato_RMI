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
        #self.coordinates = np.array([(x,y,check) for x in range(56) for y in range(28) for check in range(1)]) #array com coordenadas do mapa
        self.flagDisponivel = 0 
        self.flagParedeVert = 0
        self.flagParedeHor = 0
        self.foo = " " #inicializa todos os espaços com " "
        self.coordinates = [[self.foo for y in range(28)] for x in range(56)] #cria um array bidimensional [55][28]

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

        #print("sensor parede da frente:", (self.measures.irSensor[center_id])) 
        # print("sensor parede da direita:", (self.measures.irSensor[right_id]))
        # print("sensor parede da esquerda:", (self.measures.irSensor[left_id]))
        # print("\n")
        #print(self.measures.compass)

        if self.contadorCiclos == 0:
            self.contadorCiclos+=1
            self.firstPosX =self.measures.x-28 #tirar o 3 para ficar caso geral
            self.firstPosY =self.measures.y-14 #  //  o 11       //        //
            self.previousGps = [self.measures.x - self.firstPosX, self.measures.y - self.firstPosY]
        
        
        translation = {39: None, 91: None, 93: None, 44: None} #para tirar ']', '[', ',' e '""' do map.out
        with open('map.out', 'w') as outfile: 
            outfile.write(str(self.coordinates).translate(translation)) #imprime o mapa no map.out


        self.posX = self.measures.x-self.firstPosX #variavel que guarda a coordenada 
        self.posY = self.measures.y-self.firstPosY #sem ter que se estar sempre a fazer a conta
        #print("coordenada x: ", self.posX)
        #print("coordenada y: ", self.posY)
        print("compass: ", self.measures.compass)

        if self.count == 6:
            if self.viraEsq == 1:
                self.driveMotors(0.129, -0.129)   
            if self.viraDir == 1:
                self.driveMotors(-0.129, 0.129) 
            self.count = 0
            self.viraEsq = 0
            self.viraDir = 0

        elif self.viraEsq == 1:             #se a flag para virar à esquerda ==1 viraEsquerda
            self.driveMotors(-0.129, 0.129)  
            self.count +=1

        elif self.viraDir == 1 :
            self.driveMotors(0.129, -0.129) #se a flag para virar à esquerda ==1 viraDireita
            self.count +=1

        else:                               #Acabou de virar, executa o codigo normal
            
            d_x = self.posX - self.previousGps[0]   #diferença do deslocamento em xx
            d_y = self.posY - self.previousGps[1]   #diferença do deslocamento em yy

            #--------------------------DESLOCAMENTO NA HORIZONTAL--------------------------
            if(self.measures.compass >=-60 and self.measures.compass <=58) or (self.measures.compass <=-100 and self.measures.compass >=-180) or (self.measures.compass <=180 and self.measures.compass >=100):   #Se a bussola se encontra nestes graus entao o robot está na horizontal

                if (abs(d_x) < 2.0):  #Se a diferença entre a pos atual e anterior<2 apenas anda em frente pois não é o meio da célula
                    
                    #CÓDIGO PARA ORIENTAR O RATO NA HORIZONTAL COM A BUSSOLA PARA NAO ANDAR AOS S's (nao precisas de mexer aqui)
                    if (self.measures.compass >= -2 and self.measures.compass <0) or (self.measures.compass>= 179 and self.measures.compass<180):   #esta ligeiramente inclinado para a direita
                        self.driveMotors(0.13, 0.15)
                    elif (self.measures.compass <=2 and self.measures.compass > 0) or (self.measures.compass> -180 and self.measures.compass<=-179):  #esta ligeiramente inclinado para a esquerda 
                        self.driveMotors(0.15, 0.13)
                    else:
                        self.driveMotors(0.135, 0.135)

                else:   #Rato encontra-se no centro de uma nova celula (na horizontal) => retirar conclusões
                    
                    print("Estou no meio duma célula horizontal\n Posso tirar conclusoes dos censores\n Eis a minha posição: x=" + str(self.posX) + " y=" + str(self.posY) + "\n")
                    self.previousGps = [round(self.posX), round(self.posY)] #atualiza a posição anterior 
                    self.driveMotors(-0.15, -0.15)                          #valores para a inércia das rodas
                    
                    #TODO
                    # ---VERIFICAÇÃO DOS CENSORES---
                    # POR FLAGS NO ARRAY COM AS RESPETIVAS INFOS RETIRADAS DOS SENSORES  
                    if self.measures.compass <= 10 and self.measures.compass >= -10: #se estiver virado para norte (direita)
                        if abs(self.measures.irSensor[left_id] - self.measures.irSensor[right_id]) < 0.25: #se tiver parede dos dois lados
                            self.coordinates[round(self.posX)][round(self.posY)+1] = "-"
                            self.coordinates[round(self.posX)][round(self.posY)-1] = "-"
                        elif (self.measures.irSensor[left_id] > self.measures.irSensor[right_id]): #se tiver parede á esquerda
                            self.coordinates[round(self.posX)][round(self.posY)+1] = "-"
                        elif (self.measures.irSensor[right_id] > self.measures.irSensor[left_id]): #se tiver parede á direita not sure
                            self.coordinates[round(self.posX)][round(self.posY)-1] = "-"

                    elif abs(self.measures.compass) <= 180 and abs(self.measures.compass) >= 170: #se estiver virado para Sul (esquerda)
                        if abs(self.measures.irSensor[left_id] - self.measures.irSensor[right_id]) < 0.25: #se tiver parede dos dois lados
                            self.coordinates[round(self.posX)][round(self.posY)+1] = "-"
                            self.coordinates[round(self.posX)][round(self.posY)-1] = "-"
                        elif (self.measures.irSensor[left_id] > self.measures.irSensor[right_id]): #se tiver parede á esquerda
                            self.coordinates[round(self.posX)][round(self.posY)+1] = "-"
                        elif (self.measures.irSensor[right_id] > self.measures.irSensor[left_id]): #se tiver parede á direita
                            self.coordinates[round(self.posX)][round(self.posY)-1] = "-"
                     
                    
                    #decide para onde vai conforme valores dos sensores
                    #TODO 
                    #decide para onde vai conforme valores dos sensores e espaços nao visitados

                    if self.measures.irSensor[center_id] < 1/0.7:   #Tem a possibilidade de ir em frente pois nao existe parede
                        if self.measures.compass <= 10 and self.measures.compass >= -10: #se estiver virado para a direita
                            self.coordinates[round(self.posX)+1][round(self.posY)] = "x"
                            self.driveMotors(0.15,0.15)  
                        else: #se estiver virado para a esquerda
                            self.coordinates[round(self.posX)-1][round(self.posY)] = "x"
                            self.driveMotors(0.15,0.15)

                    elif(self.measures.irSensor[left_id])< 1/0.7:   #Tem a possibilidade de ir para a esquerda pois á esquerda nao tem parede
                        if self.measures.compass <= 10 and self.measures.compass >= -10: #se estiver virado para a direita
                            self.coordinates[round(self.posX)][round(self.posY)+1] = "x"
                            self.driveMotors(-0.129, 0.129)
                            self.viraEsq = 1
                        else: #se estiver virado para a esquerda
                            self.coordinates[round(self.posX)][round(self.posY)-1] = "x"
                            self.driveMotors(-0.129, 0.129)
                            self.viraEsq = 1

                    elif (self.measures.irSensor[right_id])< 1/0.7: #Tem a possibilidade de ir para a direita pois á direita nao tem parede
                        if self.measures.compass <= 10 and self.measures.compass >= -10: #se estiver virado para a direita
                            self.coordinates[round(self.posX)][round(self.posY)-1] = "x"
                            self.driveMotors(0.129, -0.129)
                            self.viraDir = 1
                        else: #se estiver virado para a esquerda
                            self.coordinates[round(self.posX)][round(self.posY)+1] = "x"
                            self.driveMotors(0.129, -0.129)
                            self.viraDir = 1
                            

            #--------------------------DESLOCAMENTO NA VERTICAL--------------------------            
            elif (self.measures.compass >=60 and self.measures.compass <=120) or (self.measures.compass >=-120 and self.measures.compass <=-60):  #encontra-se na vertical   
                
                if ( abs(d_y) < 2.0):  #Se a diferença entre a pos atual e anterior<2 apenas anda em frente pois não é o meio da célula
                    
                    #CÓDIGO PARA ORIENTAR O RATO NA VERTICAL COM A BUSSOLA PARA NAO ANDAR AOS S's (nao precisas de mexer aqui)
                    if (self.measures.compass >= 88 and self.measures.compass <90) or (self.measures.compass<-90 and self.measures.compass>=-92):   #esta ligeiramente inclinado para a direita
                        self.driveMotors(0.13, 0.15)
                    elif (self.measures.compass <=93 and self.measures.compass > 90) or (self.measures.compass<= -88 and self.measures.compass>-90):  #esta ligeiramente inclinado para a esquerda 
                        self.driveMotors(0.15, 0.13)
                    else:
                        self.driveMotors(0.135, 0.135)
                    
                else:   #Rato encontra-se no centro de uma nova celula (na vertical) => retirar conclusões
                    
                    print("Estou no meio duma célula vertical \nPosso tirar conclusoes dos censores\n Eis a minha posição: x=" + str(self.posX) + " y=" + str(self.posY) + "\n")
                    self.previousGps = [round(self.posX), round(self.posY)] #atualiza a posição anterior
                    self.driveMotors(-0.15, -0.15)
                    
                    #TODO
                    # ---VERIFICAÇÃO DOS CENSORES---
                    # POR FLAGS NO ARRAY COM AS RESPETIVAS INFOS RETIRADAS DOS SENSORES
                    if self.measures.compass <= 100 and self.measures.compass >= 80: #se estiver virado para cima
                        if abs(self.measures.irSensor[left_id] - self.measures.irSensor[right_id]) < 0.25: #se tiver parede dos dois lados
                            self.coordinates[round(self.posX)+1][round(self.posY)] = "|"
                            self.coordinates[round(self.posX)-1][round(self.posY)] = "|"
                        elif (self.measures.irSensor[left_id] > self.measures.irSensor[right_id]): #se tiver parede á esquerda
                            self.coordinates[round(self.posX)-1][round(self.posY)] = "|"
                        elif (self.measures.irSensor[right_id] > self.measures.irSensor[left_id]): #se tiver parede á direita not sure
                            self.coordinates[round(self.posX)+1][round(self.posY)] = "|"

                    elif abs(self.measures.compass) <= 180 and abs(self.measures.compass) >= 170: #se estiver virado para baixo
                        if abs(self.measures.irSensor[left_id] - self.measures.irSensor[right_id]) < 0.25: #se tiver parede dos dois lados
                            self.coordinates[round(self.posX)+1][round(self.posY)] = "|"
                            self.coordinates[round(self.posX)-1][round(self.posY)] = "|"
                        elif (self.measures.irSensor[left_id] > self.measures.irSensor[right_id]): #se tiver parede á esquerda
                            self.coordinates[round(self.posX+1)][round(self.posY)] = "|"
                        elif (self.measures.irSensor[right_id] > self.measures.irSensor[left_id]): #se tiver parede á direita
                            self.coordinates[round(self.posX)-1][round(self.posY)] = "|"
                    #decide para onde vai conforme valores dos sensores
                    #TODO 
                    #decide para onde vai conforme valores dos sensores e espaços nao visitados

                    if self.measures.irSensor[center_id] < 1/0.7:   #Tem a possibilidade de ir em frente pois nao existe parede
                        if self.measures.compass <= 100 and self.measures.compass >= 80: #se estiver virado para cima
                            self.coordinates[round(self.posX)][round(self.posY)+1] = "x"
                            self.driveMotors(0.15,0.15)  
                        else: #se estiver virado para baixo
                            self.coordinates[round(self.posX)][round(self.posY)-1] = "x"
                            self.driveMotors(0.15,0.15) 

                    elif(self.measures.irSensor[left_id])< 1/0.7:   #Tem a possibilidade de ir para a esquerda pois á esquerda nao tem parede
                        if self.measures.compass <= 100 and self.measures.compass >= 80: #se estiver virado para cima
                            self.coordinates[round(self.posX)-1][round(self.posY)] = "x"
                            self.driveMotors(-0.129, 0.129)
                            self.viraEsq = 1
                        else: #se estiver virado para baixo
                            self.coordinates[round(self.posX+1)][round(self.posY)] = "x"
                            self.driveMotors(-0.129, 0.129)
                            self.viraEsq = 1

                    elif (self.measures.irSensor[right_id])< 1/0.7: #Tem a possibilidade de ir para a direita pois á direita nao tem parede
                        if self.measures.compass <= 100 and self.measures.compass >= 80: #se estiver virado para cima
                            self.coordinates[round(self.posX)+1][round(self.posY)] = "x"
                            self.driveMotors(0.129, -0.129)
                            self.viraDir = 1
                        else: #se estiver virado para baixo
                            self.coordinates[round(self.posX)-1][round(self.posY)] = "x"
                            self.driveMotors(0.129, -0.129)
                            self.viraDir = 1


                    
            
            #TODO Se estiver num beco voltar a trás
            #print(previousGps, self.measures.compass)

            #if self.measures.irSensor<=2 :  #Pode andar em frente
            
           
       
        
        
        #DEPOIS EXECUTAR A FUNÇÃO FINISH

        #posicao central do 55,27 e 28,14
        #x,y, e check -> check a 0 quer dizer que nao passou na celula, check a 1 quer dizer que passou
        

        # indInicial = int(self.posX)*27+int(self.posX) #valor de x -> posiçao no array
        # indFinal = indInicial + 27
        # intervalo = self.coordinates[indInicial:indFinal+1] #intervalo de valores possiveis de y, correspondentes a esse valor de x
        # for i in range(len(intervalo)):
        #     if intervalo[i][1] == int(self.posY): #se o valor for igual á coordenada y
        #         if self.flagDisponivel == 1: 
        #             intervalo[i][2] = 1 #esse valor adquire o valor 1, quer dizer que é celula x
        #         elif self.flagParedeVert == 1: 
        #             intervalo[i][2] = 2 #esse valor adquire 2, quer dizer que é parede vertical |
        #         elif self.flagParedeHor == 1:
        #             intervalo[i][2] = 3 #esse valor adquire 3, quer dizer que é parede horizontal -
        #         else:
        #             intervalo[i][2] = 4 #esse valor adquire 4, quer dizer que unknown ""
        #             print("check:", intervalo[i])
        
        #print(self.coordinates)
        


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
