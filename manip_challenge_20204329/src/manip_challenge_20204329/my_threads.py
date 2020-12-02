class StudentThread(Thread):
    def __init__(self,host,port,sock,id):
        Thread.__init__(self)
        self.host = host
        self.port = port
        self.client_sock = sock
        self.id = id
        print('[Student({}, {})]: connected'.format(self.host, self.port))

    def run(self):
        while True:
            global tthreads
            data = self.client_sock.recv(BUFF_SIZE)
            tthreads[-1].send(data)