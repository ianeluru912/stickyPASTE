import struct
from map import TileType
from piso import Piso
from math import pi as PI
class Comm:
    def __init__(self, emitter, receiver, robot):
        self.emitter = emitter
        self.receiver = receiver
        self.robot=robot

    def sendToken(self, pos1, pos2, letra, rob):
        let = bytes(letra, 'utf-8')  
        mensaje = struct.pack("i i c i", pos1, pos2, let,rob) 
        self.emitter.send(mensaje)

    def getGameScoreAndtimeRemaining(self):

        message = struct.pack('c', 'G'.encode()) # message = 'G' for game information
        self.emitter.send(message) # send message
        gs=None
        tr=None
        rtr=None
        self.robot.robot.step()
        self.robot.robot.step()

        if self.receiver.getQueueLength() > 0: # If receiver queue is not empty
            # recorro hasta quedarme sólo con el último
            while self.receiver.getQueueLength() > 1:
                self.receiver.nextPacket()
            receivedData = self.receiver.getBytes()
            lenReceivedData = len(receivedData)
            if lenReceivedData==16: # If received data is of length 16
                # De casualidad descubrimos que también devuelve el tiempo del mundo real restante
                tup = struct.unpack('c f i i', receivedData) # Parse data into char, float, int, int
                if tup[0].decode("utf-8") == 'G':
                    gs=tup[1] #game score
                    tr=tup[2] #time remaining
                    rtr=tup[3] #real time remaining
            self.receiver.nextPacket() # Discard the current data packet
            
        return gs, tr, rtr

    def sendMap(self, rep):
     ## Get shape
        s = rep.shape
        ## Get shape as bytes
        s_bytes = struct.pack('2i',*s)

        ## Flattening the matrix and join with ','
        flatMap = ','.join(rep.flatten())
        ## Encode
        sub_bytes = flatMap.encode('utf-8')

        ## Add togeather, shape + map
        a_bytes = s_bytes + sub_bytes

        ## Send map data
        self.emitter.send(a_bytes)

        #STEP3 Send map evaluate request
        map_evaluate_request = struct.pack('c', b'M')
        self.emitter.send(map_evaluate_request)
    
    def sendExit(self):
        ## Exit message
        exit_mes = struct.pack('c', b'E')
        self.emitter.send(exit_mes)

    def llegueAVioleta(self, fila, columna):
        print(f"Yo {self.robot.yo} Mando mensaje V con {fila}, {columna}")
        message = struct.pack('c i i i i', 'V'.encode(), fila, columna, self.robot.otro, 42) 
        self.emitter.send(message)

    def quedateAhi(self, fila, columna):
        print(f"Yo {self.robot.yo} Mando mensaje Q con {fila}, {columna}")
        message = struct.pack('c i i i i', 'Q'.encode(), fila, columna, self.robot.otro, 42) 
        self.emitter.send(message)

    def yaTerminamos(self, fila, columna):
        print(f"Yo {self.robot.yo} Mando mensaje Y con {fila}, {columna}")
        message = struct.pack('c i i i i', 'Y'.encode(), fila, columna, self.robot.otro, 42) 
        self.emitter.send(message)

    def andaPalla(self, fila, columna, rob):
        print(f"Yo {self.robot.yo} Mando mensaje A con {fila}, {columna}")
        message = struct.pack('c i i i i', 'A'.encode(), fila, columna, rob, 42) 
        self.emitter.send(message)

    def recibirMensajes(self):
        if self.receiver.getQueueLength() > 0: # If receiver queue is not empty
            receivedData = self.receiver.getBytes()
 
            while self.receiver.getQueueLength()> 0 and len(receivedData)!=20:
                # print(len(receivedData))
                self.receiver.nextPacket()
                receivedData = self.receiver.getBytes()
            print("LArgo: ",len(receivedData))
            if self.receiver.getQueueLength()>0:
                paq=struct.unpack("c i i i i", receivedData)
                self.receiver.nextPacket()
                mensaje=paq[0].decode("utf-8")
                col=paq[1]
                fila=paq[2]
                destino=paq[3]
                print(f"Yo {self.robot.yo} Recibi {mensaje} con {col} y {fila} y {destino}")
                if destino==self.robot.yo:
                    if mensaje=="V":
                        self.robot.violetasDelOtro.append((col, fila))
                        print(f"Yo soy {self.robot.yo} Las violetas del otro tienen ", self.robot.violetasDelOtro)
                        # Ver qué hacer si los dos ya vieron los dos
                        if len(self.robot.violetasDelOtro)==2 and len(self.robot.misVioletas)==2:
                            
                            # sort a list of tuples by 1st item and then 2nd item
                            self.robot.violetasDelOtro.sort(key = lambda x: (x[0], x[1]))
                            self.robot.misVioletas.sort(key = lambda x: (x[0], x[1]))
                            difCol=self.robot.violetasDelOtro[0][0]-self.robot.misVioletas[0][0]
                            difFila=self.robot.violetasDelOtro[0][1]-self.robot.misVioletas[0][1]
                            print(f" Yo {self.robot.yo} Violetas del otro",self.robot.violetasDelOtro)
                            print(f" Yo {self.robot.yo} Mis Violetas", self.robot.misVioletas)
                            print("Donde me dijo que estaba el otro en su sistema de referencia", col, fila)
                            colOtro=col-difCol
                            filaOtro=fila-difFila
                            # Obtengo el camino que tengo entre mi posición y la primera violeta
                            caminoAPrimera=self.robot.navigator.pathHasta(self.robot.misVioletas[0][0], self.robot.misVioletas[0][1])
                            # if (colOtro, filaOtro) is in the keys of caminoAPrimera
                            print(caminoAPrimera)
                            posicion=self.robot.map.gridToPosition(colOtro, filaOtro)
                            posMT=self.robot.navigator.positionToMiniGrid(posicion)
                            print(f"posMT {posMT}")
                            if (posMT[0], posMT[1]) in caminoAPrimera:
                                print(f"{self.robot.otro} está en mi camino para llegar al violeta 1. Lo mando a él")
                                self.andaPalla(self.robot.violetasDelOtro[0][0], self.robot.violetasDelOtro[0][1], self.robot.otro)
                                self.robot.delay(1000)
                                self.robot.moveToPoint(self.robot.map.gridToPosition(self.robot.misVioletas[1][0], self.robot.misVioletas[1][1]), True)
                                self.robot.parar()
                                self.robot.delay(6000)
                                self.anulaVecinas()
                                self.yaTerminamos(colOtro, filaOtro)
                                self.robot.ZonaTerminada=False
                                self.robot.violetasDelOtro=[]
                                self.robot.misVioletas=[]

                            else:
                                self.quedateAhi(col, fila)
                                self.robot.moveToPoint(self.robot.map.gridToPosition(self.robot.misVioletas[1][0], self.robot.misVioletas[1][1]), True)
                                self.robot.parar()
                                self.robot.delay(6000)
                                self.anulaVecinas()
                                self.yaTerminamos(colOtro, filaOtro)
                                self.robot.ZonaTerminada=False
                                self.robot.violetasDelOtro=[]
                                self.robot.misVioletas=[]


                    elif mensaje=="Q":
                        self.robot.parar()
                        while self.robot.step() != -1:
                            if self.robot.zonaTerminada:
                                print("Pasamos a la otra zona")
                                self.robot.ZonaTerminada=False
                                self.robot.violetasDelOtro=[]
                                self.robot.misVioletas=[]
                                break
                    elif mensaje=="Y":
                        self.robot.ZonaTerminada=True
                    elif mensaje=="A":
                        self.robot.moveToPoint(self.robot.map.gridToPosition(col, fila), True)
                        self.robot.parar()
                        self.robot.delay(6000)
                        self.anulaVecinas()
                        self.robot.ZonaTerminada=False
                        self.robot.violetasDelOtro=[]
                        self.robot.misVioletas=[]

    def anulaVecinas(self):
        posicion=self.robot.map.positionToGrid(self.robot.position)
        col=posicion[0]
        fila=posicion[1]
        tile=self.robot.map.getTileAt(col, fila)
        tileN=self.robot.map.getTileAt(col, fila-1)
        tileS=self.robot.map.getTileAt(col, fila+1)
        tileE=self.robot.map.getTileAt(col+1, fila)
        tileO=self.robot.map.getTileAt(col-1, fila)
        shift = self.robot.lidar.rotToLidar(self.robot.rotation)
        lid = self.robot.lidar.rangeImage[shift:] + self.robot.lidar.rangeImage[:shift]
        if lid[0]>0.1:
            tileS.type=TileType.BLACK_HOLE
            tile.south=[1,1,1]
        else:
            tile.south=[0,0,0]
        if lid[128]>0.1:
            tileO.type=TileType.BLACK_HOLE
            tile.oeste=[1,1,1]
        else:
            tile.oeste=[0,0,0]
        if lid[256]>0.1:
            tileN.type=TileType.BLACK_HOLE
            tile.norte=[1,1,1]
        else:
            tile.norte=[0,0,0]
        if lid[384]>0.1:
            tileE.type=TileType.BLACK_HOLE
            tile.este=[1,1,1]
        else:
            tile.este=[0,0,0]
        
        for i in range(0, 4):
            b, g, r, _ = self.robot.colorSensor.getImage()
            m = Piso(r, g, b)
            if m.checkpoint():
                self.robot.avanzar(0.12, True)
                break
            else:
                self.robot.girar(PI/2)



        
                