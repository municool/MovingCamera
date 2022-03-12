import http.server as http
import RPi.GPIO as GPIO
import asyncio
import websockets
import socketserver
import multiprocessing
import cv2
import sys
import time
import numpy
from datetime import datetime as dt

# Keep track of our processes
PROCESSES = []

current_X_axis = 90
current_Y_axis = 90

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)


servoX = GPIO.PWM(11, 50)  # pin 11 for servoX, pulse 50Hz
servoY = GPIO.PWM(12, 50)  # pin 12 for servoY, pulse 50Hz
servoX.start(0)
servoY.start(0)


def setServo(servo, angle):
    servo.ChangeDutyCycle(2+(angle/18))
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)


def cleanUpServos():
    servoX.ChangeDutyCycle(7)  # Set to 90
    time.sleep(0.5)
    servoX.ChangeDutyCycle(0)

    servoY.ChangeDutyCycle(7)  # Set to 90
    time.sleep(0.5)
    servoY.ChangeDutyCycle(0)

    servoX.stop()
    servoY.stop()


def log(message):
    print("[LOG] " + str(dt.now()) + " - " + message)


def get_step_size(moving_distance, max_distance):
    return int(numpy.interp(moving_distance, [1, max_distance], [1,5]))


def move_camera(x, y, w, h, dimensions):
    centerOfScreen = (dimensions[1] / 2, dimensions[0] / 2) # x , y
    if((x + (w/2)) < centerOfScreen[0]):
        # face is on the left side of the screen
        log("move camera to the left")
        current_X_axis =- get_step_size(centerOfScreen[0] - (x + (w/2)), centerOfScreen[0])
        if(current_X_axis < 0):
            current_X_axis = 0

    if((x + (w/2)) > centerOfScreen[0]):
        # face is on the right side of the screen
        log("move camera to the right")
        current_X_axis =+ get_step_size((x + (w/2)) - centerOfScreen[0], centerOfScreen[0])
        if(current_X_axis > 180):
            current_X_axis = 180

    if((y + (h/2)) < centerOfScreen[1]):
        # face is on the top side of the screen
        log("move camera up")
        current_Y_axis =- get_step_size(centerOfScreen[0] - (y + (h/2)), centerOfScreen[1])
        if(current_Y_axis < 0):
            current_Y_axis = 0

    if((y + (h/2)) > centerOfScreen[1]):
        # face is on the bottom of the screen
        log("move camera down")        
        current_Y_axis = + get_step_size((y + (h/2)) - centerOfScreen[1], centerOfScreen[1])
        if(current_Y_axis > 180):
            current_Y_axis = 180

    # print("X: " + str(x + (w/2)) + ", Y: " + str(y + (h/2)))
    # print("Axis_X: " + str(current_X_axis) + ", Axis_Y: " + str(current_Y_axis))
    setServo(servoX, current_X_axis)
    setServo(servoY, current_Y_axis)
    # move camera to the current axises


def camera(man):
    log("Starting camera")
    vc = cv2.VideoCapture(0)
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    if vc.isOpened():
        r, f = vc.read()
    else:
        r = False

    while r:
        cv2.waitKey(20)
        r, f = vc.read()
        # f = cv2.resize(f, (640, 480))

        gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)
        # Mark the found faces
        for (x, y, w, h) in faces:
            cv2.rectangle(f, (x, y), (x + w, y + h), (255, 0, 0), 5)
            move_camera(x, y, w, h, f.shape)

        #Encode picture and send it
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 65]
        man[0] = cv2.imencode('.jpg', f, encode_param)[1]


# HTTP server handler
def server():
    server_address = ('0.0.0.0', 8000)
    if sys.version_info[1] < 7:
        class ThreadingHTTPServer(socketserver.ThreadingMixIn, http.HTTPServer):
            pass
        httpd = ThreadingHTTPServer(server_address, http.SimpleHTTPRequestHandler)
    else:
        httpd = http.ThreadingHTTPServer(server_address, http.SimpleHTTPRequestHandler)
    log("Server started")
    httpd.serve_forever()

def socket(man):
    async def handler(websocket, path):
        log("Socket opened")
        try:
            while True:
                await asyncio.sleep(0.033) # 30 fps
                await websocket.send(man[0].tobytes())
        except websockets.exceptions.ConnectionClosed:
            log("Socket closed")

    log("Starting socket handler")

    start_server = websockets.serve(ws_handler=handler, host='0.0.0.0', port=8585)

    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()


def main():
    manager = multiprocessing.Manager()
    lst = manager.list()
    lst.append(None)

    http_server = multiprocessing.Process(target=server)
    socket_handler = multiprocessing.Process(target=socket, args=(lst,))
    camera_handler = multiprocessing.Process(target=camera, args=(lst,))

    PROCESSES.append(camera_handler)
    PROCESSES.append(http_server)
    PROCESSES.append(socket_handler)

    for p in PROCESSES:
        p.start()
    # Wait forever
    while True:
        pass

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        cleanUpServos()
        for p in PROCESSES:
            p.terminate()