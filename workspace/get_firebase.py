import time
from firebase import firebase

firebase = firebase.FirebaseApplication('https://mobile-robot-6269b-default-rtdb.firebaseio.com/', None)
result = firebase.get('/Control', None)

while True:
    result = firebase.get('/Control', None)
    print(result)
