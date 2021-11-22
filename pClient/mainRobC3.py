# coding=utf-8
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
    global offset
    global dif
    

    
    dif = 0
    offset = [0,0]
    
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
        self.flagDisponivel = 0 
        self.flagParedeVert = 0
        self.flagParedeHor = 0
        self.foo = "-" #inicializa todos os espaços com "-"
        self.coordinates = [[self.foo for x in range(55)] for y in range(27)] #cria um array bidimensional [56][27]
        self.walls = []
        self.objetivosPassados= [(27, 13)]
        self.nObjetivos = 0
        self.counterObjetivos = 0

        
        
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


        if self.contadorCiclos == 0: #CICLO 0
            self.contadorCiclos+=1
            self.firstPosX =self.measures.x-27 #tirar o 3 para ficar caso geral
            self.firstPosY =self.measures.y-14 #  //  o 11       //        //
            self.posX = 27
            self.posY = 13
            self.previousGps = [self.measures.x - self.firstPosX, self.measures.y -self.firstPosY]
            if self.measures.compass <= 10 and self.measures.compass >= -10: #mapeamento da posicao adjacente ao I
                self.coordinates[round(self.posY)][round(self.posX+1)] = "X"
            elif abs(self.measures.compass) <= 180 and abs(self.measures.compass) >= 170: 
                self.coordinates[round(self.posY)][round(self.posX-1)] = "X"
            self.nObjetivos = int(self.nBeacons)-1    #-1 PQE O INICIO TAMBEM CONTA COMO BEACON
            
        
        # with open('path.out', 'w') as outfile: 
        #     for i in range(27):
        #         for j in range(55):
        #             outfile.write(self.coordinates[i][j])
        #             #print(self.coordinates[i][j])
        #         outfile.write("\n")
        #print("\n".join(["".join([x for x in row])for row in self.coordinates]))
        
        self.coordinates[13][27] = "X" #posicao inicial
        self.posX = self.measures.x-self.firstPosX #variavel que guarda a coordenada 
        self.posY = self.firstPosY - self.measures.y     #sem ter que se estar sempre a fazer a conta


        if self.count == 6 or self.countReverte == 12:
            if self.viraEsq == 1:
                self.driveMotors(0.129, -0.129)   
            if self.viraDir == 1:
                self.driveMotors(-0.129, 0.129) 
            if self.reverte == 1:
                self.driveMotors(-0.129, 0.129) # ou seja ele reverte para a direita
            
            self.countReverte = 0
            self.reverte = 0
            self.count = 0
            self.viraEsq = 0
            self.viraDir = 0

        elif self.viraEsq == 1:             #se a flag para virar à esquerda ==1 viraEsquerda
            self.driveMotors(-0.129, 0.129)  
            self.count +=1

        elif self.viraDir == 1 :
            self.driveMotors(0.129, -0.129) #se a flag para virar à esquerda ==1 viraDireita
            self.count +=1

        elif self.reverte == 1 :
            self.driveMotors(0.129, -0.129) #se a flag para reverter ==1 
            self.countReverte +=1
        else:                               #Acabou de virar, executa o codigo normal
            
            d_x = self.posX - self.previousGps[0]   #diferença do deslocamento em xx
            d_y = self.posY - self.previousGps[1]   #diferença do deslocamento em yy

            #--------------------------DESLOCAMENTO NA HORIZONTAL--------------------------
            if(self.measures.compass >=-60 and self.measures.compass <=58) or (self.measures.compass <=-100 and self.measures.compass >=-180) or (self.measures.compass <=180 and self.measures.compass >=100):   #Se a bussola se encontra nestes graus entao o robot está na horizontal

                if (abs(d_x) < 2.0):  #Se a diferença entre a pos atual e anterior<2 apenas anda em frente pois não é o meio da célula
                    
                    #CÓDIGO PARA ORIENTAR O RATO NA HORIZONTAL COM A BUSSOLA PARA NAO ANDAR AOS S's (nao precisas de mexer aqui)
                    if (self.measures.compass >= -4 and self.measures.compass <0) or (self.measures.compass>= 176 and self.measures.compass<180):   #esta ligeiramente inclinado para a direita
                        self.driveMotors(0.13, 0.15)
                    elif (self.measures.compass <=4 and self.measures.compass > 0) or (self.measures.compass> -180 and self.measures.compass<=-176):  #esta ligeiramente inclinado para a esquerda 
                        self.driveMotors(0.15, 0.13)
                    else:
                        self.driveMotors(0.15, 0.15)

                else:   #Rato encontra-se no centro de uma nova celula (na horizontal) => retirar conclusões
                    self.contadorCiclos +=1
                    self.previousGps = [round(self.posX), round(self.posY)]     #atualiza a posição anterior
                    self.driveMotors(-0.15, -0.15)                              #valores para a inércia das rodas
                    self.coordinates[round(self.posY)][round(self.posX)] = "X"  #a posicao onde se encontra esta vazia (x)
                    
                    # ---VERIFICAÇÃO DOS CENSORES---
                    # POR FLAGS NO ARRAY COM AS RESPETIVAS INFOS RETIRADAS DOS SENSORES  
                    if self.measures.compass <= 10 and self.measures.compass >= -10:    #se estiver virado para norte (direita)
                        if self.measures.irSensor[center_id] > 1.2:                     #se tiver parede à frente
                            self.coordinates[round(self.posY)][round(self.posX+1)] = "-"
                        else: 
                            self.coordinates[round(self.posY)][round(self.posX+1)] = "X"
                        if self.measures.irSensor[left_id] > 1.2:   #se tiver parede á esquerda
                            self.coordinates[round(self.posY)-1][round(self.posX)] = "-"
                        else:
                            self.coordinates[round(self.posY)-1][round(self.posX)] = "X" #se nao tiver parede quer dizer q é vazia
                        if self.measures.irSensor[right_id] > 1.2:  #se tiver parede á direita
                            self.coordinates[round(self.posY)+1][round(self.posX)] = "-"
                        else: 
                            self.coordinates[round(self.posY)+1][round(self.posX)] = "X" #se nao tiver parede quer dizer q ta vazia

                    elif abs(self.measures.compass) <= 180 and abs(self.measures.compass) >= 170: #se estiver virado para Sul (esquerda)
                        if self.measures.irSensor[center_id] > 1.2: #se tiver parede à frente
                            self.coordinates[round(self.posY)][round(self.posX-1)] = "-"
                        else: 
                            self.coordinates[round(self.posY)][round(self.posX-1)] = "X"
                        if self.measures.irSensor[left_id] > 1.2:   #se tiver parede á esquerda
                            self.coordinates[round(self.posY)+1][round(self.posX)] = "-"
                        else:
                            self.coordinates[round(self.posY)+1][round(self.posX)] = "X"
                        if self.measures.irSensor[right_id] > 1.2:  #se tiver parede á direita
                            self.coordinates[round(self.posY)-1][round(self.posX)] = "-"
                        else:  
                            self.coordinates[round(self.posY)-1][round(self.posX)] = "X"
                     
                    #verifica se é um beacon
                    if self.measures.ground >0: #significa que está em cima de um checkpoint
                        
                        #chamar o a* calcular o caminho, e escrever logo no ficheiro
                        
                      
                        for i in range(len(self.coordinates)):#LINHAS
                            for j in range(len(self.coordinates[i])):
                                if str(self.coordinates[i][j]) == "-":   #verificar se é ij ou ji 
                                    self.walls.append((j, i))
                                
                        self.objetivosPassados.append((round(self.posX), abs(round(self.posY))))
                        self.objetivosPassados = list(set([i for i in self.objetivosPassados]))    #remove duplicates (para o caso de passar varias vezes no mesmo checkpoint) 

                    #decide para onde vai conforme valores dos sensores
                    
                    if (self.measures.irSensor[center_id]< 1/0.72 and self.measures.compass <= 10 and self.measures.compass >= -10 and self.coordinates[round(self.posY)][round(self.posX)+2] != "X")\
                     or (self.measures.irSensor[center_id]< 1/0.72 and (self.measures.compass > 170 or self.measures.compass < -170) and self.coordinates[round(self.posY)][round(self.posX)-2] != "X"):   #Tem a possibilidade de ir em frente pois nao existe parede e não tem parede visitada
                        self.driveMotors(0.15,0.15)

                    elif(self.measures.irSensor[left_id]< 1/0.72 and self.measures.compass <= 10 and self.measures.compass >= -10 and self.coordinates[round(self.posY)-2][round(self.posX)] != "X")\
                     or (self.measures.irSensor[left_id]< 1/0.72 and (self.measures.compass > 170 or self.measures.compass < -170) and self.coordinates[round(self.posY)+2][round(self.posX)] != "X"):     #Tem a possibilidade de ir para a esquerda pois á esquerda nao tem parede e nao foi visitada
                        self.driveMotors(-0.129, 0.129)
                        self.viraEsq = 1

                    elif(self.measures.irSensor[right_id]< 1/0.72 and self.measures.compass <= 10 and self.measures.compass >= -10 and self.coordinates[round(self.posY)+2][round(self.posX)] != "X")\
                     or (self.measures.irSensor[right_id]< 1/0.72 and (self.measures.compass > 170 or self.measures.compass < -170) and self.coordinates[round(self.posY)-2][round(self.posX)] != "X"):     #Tem a possibilidade de ir para a direita pois à direita nao tem parede e nao foi visitada
                        self.driveMotors(0.129, -0.129)
                        self.viraDir = 1

                    elif (self.measures.irSensor[right_id]>= 1/0.72 and self.measures.irSensor[center_id]>= 1/0.72 and self.measures.irSensor[left_id]>= 1/0.72):   #Está num beco, tem de inverter
                        self.driveMotors(0.129, -0.129)
                        self.reverte = 1
                   
                    else:   #As posições à volta dele já estão todas visitadas
                        
                        if (self.measures.irSensor[left_id]< 1/0.72) and (self.measures.irSensor[right_id]< 1/0.72):
                            
                            a = random.randrange(2)
                            
                            if a==0:
                                self.driveMotors(0.129, -0.129)
                                self.viraDir = 1
                            elif a==1:
                                self.driveMotors(-0.129, 0.129)
                                self.viraEsq = 1
                        elif (self.measures.irSensor[left_id]< 1/0.72):
                           
                            self.driveMotors(-0.129, 0.129)
                            self.viraEsq = 1
                        elif (self.measures.irSensor[right_id]< 1/0.72):
                            self.driveMotors(0.129, -0.129)
                            
                            self.viraDir = 1
                        elif self.measures.irSensor[center_id] < 1/0.72:
                            self.driveMotors(0.15,0.15)
                            
        
                            

            #--------------------------DESLOCAMENTO NA VERTICAL--------------------------            
            elif (self.measures.compass >=60 and self.measures.compass <=120) or (self.measures.compass >=-120 and self.measures.compass <=-60):  #encontra-se na vertical   
                
                if ( abs(d_y) < 2.0):  #Se a diferença entre a pos atual e anterior<2 apenas anda em frente pois não é o meio da célula
                    
                    #CÓDIGO PARA ORIENTAR O RATO NA VERTICAL COM A BUSSOLA PARA NAO ANDAR AOS S's (nao precisas de mexer aqui)
                    if (self.measures.compass >= 86 and self.measures.compass <90) or (self.measures.compass<-90 and self.measures.compass>=-94):   #esta ligeiramente inclinado para a direita
                        self.driveMotors(0.13, 0.15)
                    elif (self.measures.compass <=94 and self.measures.compass > 90) or (self.measures.compass<= -86 and self.measures.compass>-90):  #esta ligeiramente inclinado para a esquerda 
                        self.driveMotors(0.15, 0.13)
                    else:
                        self.driveMotors(0.15, 0.15)
                    
                else:   #Rato encontra-se no centro de uma nova celula (na vertical) => retirar conclusões
                    self.contadorCiclos +=1
                    self.previousGps = [round(self.posX), round(self.posY)] #atualiza a posição anterior
                    self.driveMotors(-0.15, -0.15)
                    self.coordinates[round(self.posY)][round(self.posX)] = "X" #a posicao onde se encontra esta vazia (x)

                    
                    # ---VERIFICAÇÃO DOS CENSORES---
                    # POR FLAGS NO ARRAY COM AS RESPETIVAS INFOS RETIRADAS DOS SENSORES
                    if self.measures.compass <= 100 and self.measures.compass >= 80: #se estiver virado para cima
                        if self.measures.irSensor[center_id] > 1.2: #se tiver parede à frente
                            self.coordinates[round(self.posY-1)][round(self.posX)] = "-"
                        else: 
                            self.coordinates[round(self.posY-1)][round(self.posX)] = "X"
                        if self.measures.irSensor[left_id] > 1.2: #se tiver parede á esquerda
                            self.coordinates[round(self.posY)][round(self.posX)-1] = "-"
                        else:
                            self.coordinates[round(self.posY)][round(self.posX)-1] = "X" 
                        if self.measures.irSensor[right_id] > 1.2: #se tiver parede á direita not sure
                            self.coordinates[round(self.posY)][round(self.posX)+1] = "-"
                        else:
                            self.coordinates[round(self.posY)][round(self.posX)+1] = "X"

                    elif abs(self.measures.compass) <= 100 and abs(self.measures.compass) >= 80: #se estiver virado para baixo
                        if self.measures.irSensor[center_id] > 1.2: #se tiver parede à frente
                            self.coordinates[round(self.posY+1)][round(self.posX)] = "-"
                        else: 
                            self.coordinates[round(self.posY+1)][round(self.posX)] = "X"
                        if self.measures.irSensor[left_id] > 1.2: #se tiver parede á esquerda
                            self.coordinates[round(self.posY)][round(self.posX+1)] = "-"
                        else:
                            self.coordinates[round(self.posY)][round(self.posX+1)] = "X"
                        if self.measures.irSensor[right_id] > 1.2: #se tiver parede á direita
                            self.coordinates[round(self.posY)][round(self.posX)-1] = "-"
                        else:
                            self.coordinates[round(self.posY)][round(self.posX)-1] = "X"
                    

                    #verifica se é um beacon
                    if self.measures.ground >0: #significa que está em cima de um checkpoint
                        
                        #chamar o a* calcular o caminho, e escrever logo no ficheiro
                        
                      
                        for i in range(len(self.coordinates)):#LINHAS
                            for j in range(len(self.coordinates[i])):
                                if str(self.coordinates[i][j]) == "-":
                                    self.walls.append((j, i))
                                
                        self.objetivosPassados.append((round(self.posX), abs(round(self.posY))))    #adiciona ao array o checkpoint atual
                        self.objetivosPassados = list(set([i for i in self.objetivosPassados]))    #remove duplicates (para o caso de passar varias vezes no mesmo checkpoint)

                    #decide para onde vai conforme valores dos sensores
                    
                    if (self.measures.irSensor[center_id]< 1/0.72 and self.measures.compass <= 100 and self.measures.compass >= 80 and self.coordinates[round(self.posY-2)][round(self.posX)] != "X")\
                     or (self.measures.irSensor[center_id]< 1/0.72 and (self.measures.compass > -100 and self.measures.compass < -80) and self.coordinates[round(self.posY)+2][round(self.posX)] != "X"):   #Tem a possibilidade de ir em frente pois nao existe parede e não foi visitada
                        self.driveMotors(0.15,0.15)

                    elif(self.measures.irSensor[left_id]< 1/0.72 and self.measures.compass <= 100 and self.measures.compass >= 80 and self.coordinates[round(self.posY)][round(self.posX)-2] != "X")\
                     or (self.measures.irSensor[left_id]< 1/0.72 and (self.measures.compass > -100 and self.measures.compass < -80) and self.coordinates[round(self.posY)][round(self.posX)+2] != "X"):     #Tem a possibilidade de ir para a esquerda pois á esquerda nao tem parede e nao foi visitada
                        self.driveMotors(-0.129, 0.129)
                        self.viraEsq = 1

                    elif(self.measures.irSensor[right_id]< 1/0.72 and self.measures.compass <= 100 and self.measures.compass >= 80 and self.coordinates[round(self.posY)][round(self.posX)+2] != "X")\
                     or (self.measures.irSensor[right_id]< 1/0.72 and (self.measures.compass > -100 and self.measures.compass < -80) and self.coordinates[round(self.posY)][round(self.posX)-2] != "X"):     #Tem a possibilidade de ir para a direita pois à direita nao tem parede e nao foi visitada
                        self.driveMotors(0.129, -0.129)
                        self.viraDir = 1

                    elif (self.measures.irSensor[right_id]>= 1/0.72 and self.measures.irSensor[center_id]>= 1/0.72 and self.measures.irSensor[left_id]>= 1/0.72):   #Está num beco, tem de inverter
                        self.driveMotors(0.129, -0.129)
                        self.reverte = 1
                    
                    else:   #As posições à volta dele já estão todas visitadas
                        #print("já visitei tudo à minha volta")
                        if (self.measures.irSensor[left_id]< 1/0.72) and (self.measures.irSensor[right_id]< 1/0.72):
                            a = random.randrange(2)
                            
                            if a==0:
                                self.driveMotors(0.129, -0.129)
                                self.viraDir = 1
                            elif a==1:
                                self.driveMotors(-0.129, 0.129)
                                self.viraEsq = 1
                        elif self.measures.irSensor[center_id] < 1/0.72:
                            self.driveMotors(0.15,0.15)
                        elif (self.measures.irSensor[left_id]< 1/0.72):
                            self.driveMotors(-0.129, 0.129)
                            self.viraEsq = 1
                        elif (self.measures.irSensor[right_id]< 1/0.72):
                            self.driveMotors(0.129, -0.129)
                            self.viraDir = 1

        if self.contadorCiclos %10 ==0:#determina A*
            self.contadorCiclos+=1
            if len(self.objetivosPassados)>1: #para guardar num array a solução
                            solucao=[]
                            #executa o A* e imprime no ficheiro 
                            for b in range(len(self.objetivosPassados)):
                                if b< len(self.objetivosPassados)-1:
                                    
                                    a = pf.AStar()
                                    a.init_grid(56, 28, self.walls, self.objetivosPassados[b], self.objetivosPassados[b+1])
                                    path = a.solve()
                                    solucao.append(path)
                                    #print(path)

                                else: #para imprimir da ultima posiçaõ adicionada aos checkpoints até ao inicio   
                                    a = pf.AStar()
                                    a.init_grid(56, 28, self.walls, self.objetivosPassados[b], (27, 13))
                                    path = a.solve()
                                    solucao.append(path)
                                    #print(path)
                            #print("objetivos passados:  " + str(self.objetivosPassados) + "\n")
                            print("Solução" + str(solucao))

                            #for x in range(len(solucao)): #TODO OBJETIVO CADA VALOR DO PATH MENOS 23,17 E IMPRIMIR VALOR SIM VALOR NAO
                                #if(x%2 == 0):
                                    #print(solucao[x])
                            with open('path.out', 'w') as outfile: 
                                for t in solucao:
                                        outfile.write(' '.join(str(s) for s in t) + '\n')
            



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
