from flask import Flask, jsonify
from flask_cors import CORS
import threading
import time
import serial

app = Flask(__name__)

CORS(app)

bluetoothSerial = serial.Serial( "COM7", baudrate=38400, timeout=3)
data = {'hmd': 0, 'tmp': 0, 'CO': 0, 'alc': 0, 'CO2': 0, 'tol': 0, 'NH4': 0, 'ace': 0}

sensor_data = {
    'Co': 250,
    'Alcohol': 100,
    'Tolueno': 50,
    'Nh4': 120,
    'Acetona': 30
}

temperature_data = {
    'current': 0,
    'min': 1000,
    'max': -1000
}

def update_temperature():
    while True:
        try:
            temperature_data['current'] = data['tmp']
            if int(float(data['tmp'])) < int(float(temperature_data['min'])):
                temperature_data['min'] = data['tmp']
            if int(float(data['tmp'])) > int(float(temperature_data['max'])):
                temperature_data['max'] = data['tmp']
            time.sleep(0.1)  # Actualiza cada 0.1 segundos
        except Exception as e:
            print("Error de lectura de temperatura")
            print(e)


# Función para actualizar los valores de los sensores periódicamente
def update_sensor_data():
    while True:
        sensor_data['Co'] = data['CO']
        sensor_data['Alcohol'] = data['alc']
        sensor_data['Tolueno'] = data['tol']
        sensor_data['Nh4'] = data['NH4']
        sensor_data['Acetona'] = data['ace']
        time.sleep(1)  # Actualiza cada 1 segundos

def read_serial():
    while True:
        try:
            if(bluetoothSerial.in_waiting > 0):
                line = bluetoothSerial.readline().decode('utf-8').strip()
                serialData = dict(x.split(':') for x in line.split(','))
                data['hmd'] = serialData['hmd']
                data['tmp'] = serialData['tmp']
                data['CO'] = serialData['CO']
                data['alc'] = serialData['alc']
                data['CO2'] = serialData['CO2']
                data['tol'] = serialData['tol']
                data['NH4'] = serialData['NH4']
                data['ace'] = serialData['ace']
                print(serialData)
                time.sleep(0.1)
        except Exception as e:
            print("Error de lectura")
            print(e)

@app.route('/')
def index():
    return "API Flask funcionando"

@app.route('/temperature', methods=['GET'])
def get_temperature_value():
    return jsonify(temperature_data)

@app.route('/gas', methods=['GET'])
def get_sensor_data():
    return jsonify(sensor_data)

@app.route('/air_quality', methods=['GET'])
def get_air_quality():
    quality = classify_air_quality(sensor_data)
    return jsonify({'quality': quality, 'combined_index': calculate_combined_air_quality(sensor_data)})

def classify_air_quality(data):
    combined_index = calculate_combined_air_quality(data)
    if combined_index <= 50:
        return 'sin riesgo'
    elif combined_index <= 100:
        return 'moderado'
    elif combined_index <= 150:
        return 'dañino'
    else:
        return 'muy dañino'

def calculate_combined_air_quality(data):
    return (int(float(data['Co'])) + int(float(data['Alcohol'])) + int(float(data['Tolueno'])) + int(float(data['Nh4'])) + int(float(data['Acetona']))) / 5

if __name__ == "__main__":
    serial_thread = threading.Thread(target=read_serial)
    serial_thread.daemon = True
    serial_thread.start()
    threading.Thread(target=update_sensor_data, daemon=True).start()
    threading.Thread(target=update_temperature, daemon=True).start()
    app.run(host='localhost', port=5000)
    
    