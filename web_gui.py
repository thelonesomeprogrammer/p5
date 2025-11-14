from flask import Flask
from serial import Serial
from threading import Thread
from queue import Queue


class Serialmanager:
    def __init__(self, port='/dev/ttyACM0', baudrate=9600, timeout=1):
        self.ser = Serial(port, baudrate=baudrate, timeout=timeout)
        self.data = Queue(10)
        self.thread = Thread(target=self.read_from_port)
        self.thread.daemon = True
        self.thread.start()

    def read_from_port(self):
        while True:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                self.data.put(line)

    def readline(self):
        return self.data.get()

    def readlines(self):
        lines = []
        while not self.data.empty():
            lines.append(self.data.get())
        return lines

port = Serialmanager()

app = Flask(__name__,static_folder='./build/', static_url_path='/')

@app.route('/')
def index():
    return app.send_static_file('index.html')

@app.route('/api/line')
def data():
    return port.readline()

@app.route('/api/lines')
def datas():
    return '\n'.join(port.readlines())

def main(args=None):
    app.config['SEND_FILE_MAX_AGE_DEFAULT'] = -1
    app.run(host='0.0.0.0', port=5000 ,debug=False)

if __name__ == '__main__':
    main()

