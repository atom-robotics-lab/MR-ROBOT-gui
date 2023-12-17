var ros = new ROSLIB.Ros({
    url: 'ws://0.0.0.0:9090'
});

var publisher = new ROSLIB.Topic({
    ros: ros,
    name: '/hello',
    messageType: 'std_msgs/String'
});

var cmdVel = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_vel',
    messageType: 'geometry_msgs/Twist'
});

var speed = 0.4;
var speedIncrement = 1.0;
var isMousePressed = false;
var isShiftKeyPressed = false;

var joystickContainer = document.getElementById('joystickContainer');
var joystickStick = document.getElementById('joystickStick');

document.getElementById('publishButton').addEventListener('click', function () {
    var message = new ROSLIB.Message({
        data: 'Hello, ROS!'
    });
    publisher.publish(message);
    console.log('Message published!');
});

document.addEventListener('keydown', function (event) {
    if (event.key.toLowerCase() === 'shift') {
        isShiftKeyPressed = true;
        console.log("Shift key is pressed....Speed is increased")
    }
});

document.addEventListener('keyup', function (event) {
    if (event.key.toLowerCase() === 'shift') {
        isShiftKeyPressed = false;
        console.log("Shift key is released....Speed reset to default")
    }
});

joystickContainer.addEventListener('mousedown', function () {
    isMousePressed = true;
    joystickContainer.addEventListener('mousemove', handleJoystickMove);
});

document.addEventListener('mouseup', function () {
    isMousePressed = false;
    joystickContainer.removeEventListener('mousemove', handleJoystickMove);
    resetJoystick();
    stopMovement();
});

function handleJoystickMove(event) {
    var rect = joystickContainer.getBoundingClientRect();
    var x = event.clientX - rect.left - joystickStick.clientWidth / 2;
    var y = event.clientY - rect.top - joystickStick.clientHeight / 2;

    // Ensure the joystick stays within the bounds of the container
    x = Math.max(0, Math.min(x, rect.width - joystickStick.clientWidth));
    y = Math.max(0, Math.min(y, rect.height - joystickStick.clientHeight));

    joystickStick.style.transform = `translate(${x}px, ${y}px)`;
    publishJoystickCommand(x, y);
}

function resetJoystick() {
    joystickStick.style.transform = 'translate(100%, 100%)';
    publishJoystickCommand(0, 0);
}

function publishJoystickCommand(x, y) {
    var normalizedX = (x - joystickContainer.clientWidth / 2) / (joystickContainer.clientWidth / 2);
    var normalizedY = (y - joystickContainer.clientHeight / 2) / (joystickContainer.clientHeight / 2);

    var linearX = -normalizedY * (isMousePressed ? speed + (isShiftKeyPressed ? speedIncrement : 0) : 0);
    var linearY = normalizedX * (isMousePressed ? speed + (isShiftKeyPressed ? speedIncrement : 0) : 0);
    var angularZ = (normalizedX * (isMousePressed ? speed + (isShiftKeyPressed ? speedIncrement : 0) : 0)) / 2;

    publishTwist(linearX, linearY, angularZ);
}

function publishTwist(linearX, linearY, angularZ) {
    var twist = new ROSLIB.Message({
        linear: {
            x: linearX,
            y: linearY,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: angularZ
        }
    });
    cmdVel.publish(twist);

    if (linearX > 0) {
        console.log('Moving forward - Publish "W"');
    } else if (linearX < 0) {
        console.log('Moving backward - Publish "S"');
    }

    if (linearY < 0) {
        console.log('Moving left - Publish "A"');
    } else if (linearY > 0) {
        console.log('Moving right - Publish "D"');
    }
}

function stopMovement() {
    console.log('Movement stopped');
    publishTwist(0, 0, 0);
}

ros.on('connection', function () {
    console.log('Connected to ROS!');
});

ros.on('error', function (error) {
    console.error('Error connecting to ROS:', error);
});

ros.on('close', function () {
    console.log('Connection to ROS closed.');
});