from flask import Flask
# from serial import Serial


class DummySerial:
    def readline(self):
        return b"700\n"
port = DummySerial()

#port = Serial('/dev/ttyACM0', baudrate=9600, timeout=1)

app = Flask(__name__,static_folder='./build/', static_url_path='/')

@app.route('/')
def index():
    return app.send_static_file('index.html')

@app.route('/data')
def data():
    return port.readline().decode('utf-8').strip()

def main(args=None):
    app.config['SEND_FILE_MAX_AGE_DEFAULT'] = -1
    app.run(host='0.0.0.0', port=5000 ,debug=False)

if __name__ == '__main__':
    main()

