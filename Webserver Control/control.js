// For Firebase JS SDK v7.20.0 and later, measurementId is optional
const firebaseConfig = {
    apiKey: "AIzaSyCp695REtcZgEzWWgpEOTgD5mwYJ5ptiJQ",
    authDomain: "mobile-robot-6269b.firebaseapp.com",
    databaseURL: "https://mobile-robot-6269b-default-rtdb.firebaseio.com",
    projectId: "mobile-robot-6269b",
    storageBucket: "mobile-robot-6269b.appspot.com",
    messagingSenderId: "1062485308313",
    appId: "1:1062485308313:web:41fad77a809a8f22e29908",
  };

// Khởi tạo Firebase
firebase.initializeApp(firebaseConfig);
var database = firebase.database();

// Lấy thông tin phần tử
var forwardButton = document.getElementById("forward");
var backwardButton = document.getElementById("backward");
var leftButton = document.getElementById("left");
var rightButton = document.getElementById("right");

// Khai báo biến để lưu trạng thái điều khiển
let controlState = { Forward: 0, Backward: 0, Left: 0, Right: 0 };

// Gán sự kiện khi nhấn giữ các button điều khiển
forwardButton.addEventListener("mousedown", function() {
    controlState.Forward = 1;
    database.ref("/Control").update(controlState);
});
backwardButton.addEventListener("mousedown", function() {
    controlState.Backward = 1;
    database.ref("/Control").update(controlState);
});
leftButton.addEventListener("mousedown", function() {
    controlState.Left = 1;
    database.ref("/Control").update(controlState);
});
rightButton.addEventListener("mousedown", function() {
    controlState.Right = 1;
    database.ref("/Control").update(controlState);
});

// Gán sự kiện khi nhả các button điều khiển
forwardButton.addEventListener("mouseup", function() {
    controlState.Forward = 0;
    database.ref("/Control").update(controlState);
});
backwardButton.addEventListener("mouseup", function() {
    controlState.Backward = 0;
    database.ref("/Control").update(controlState);
});
leftButton.addEventListener("mouseup", function() {
    controlState.Left = 0;
    database.ref("/Control").update(controlState);
});
rightButton.addEventListener("mouseup", function() {
    controlState.Right = 0;
    database.ref("/Control").update(controlState);
});