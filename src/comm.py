import struct

class Comm:
    def __init__(self, emitter, receiver, robot):
        self.emitter = emitter
        self.receiver = receiver
        self.robot=robot

    def sendToken(self, pos1, pos2, letra):
        let = bytes(letra, 'utf-8')  
        mensaje = struct.pack("i i c", pos1, pos2, let) 
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