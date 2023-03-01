import time
from firebase import firebase

firebase = firebase.FirebaseApplication('https://mobile-robot-6269b-default-rtdb.firebaseio.com/', None)

while True:
    result = firebase.get('/Control', None)
    if result["Forward"] == 1:
        print("Forward")
    elif result["Backward"] == 1:
        print("Backward")
    elif result["Left_Thuan"] == 1:
        print("Left_Thuận")
    elif result["Left_Nguoc"] == 1:
        print("Left_Ngược")
    elif result["Right_Thuan"] == 1:
        print("Right_Thuận")
    elif result["Right_Nguoc"] == 1:
        print("Right_Ngược")
    else:
        print("Stopmotor")