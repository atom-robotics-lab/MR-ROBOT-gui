var ros = new ROSLIB.Ros({
  url: "ws://localhost:9090",
});

var publisher = new ROSLIB.Topic({
  ros: ros,
  name: "/hello",
  messageType: "std_msgs/String",
});

var cmdVel = new ROSLIB.Topic({
  ros: ros,
  name: "/cmd_vel",
  messageType: "geometry_msgs/Twist",
});

var speed = 0.4;
var speedIncrement = 4;
var shiftKeyPressed = false;
var isMoving = false;
var boredTimer = null;

var odomTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/odom",
  messageType: "nav_msgs/Odometry",
});

var lastReceivedTime = 0;
var lastPosition = null;

odomTopic.subscribe(function (message) {
  var currentTime = new Date().getTime();
  if (currentTime - lastReceivedTime > 60000) {
    console.log("Received pose: ", message.pose.pose.position);

    if (isCollision(message.pose.pose.position)) {
      playGif("dead");
    }

    lastReceivedTime = currentTime;
    lastPosition = message.pose.pose.position;
  }
});

function isCollision(currentPosition) {
  if (lastPosition && lastPosition.x === currentPosition.x) {
    // Assuming collision if x position remains the same for 1 minute
    return true;
  }
  return false;
}

playGif("default");

document.getElementById("publishButton").addEventListener("click", function () {
  var message = new ROSLIB.Message({
    data: "Hello, ROS!",
  });

  publisher.publish(message);
  console.log("Message published!");
});

document.onkeydown = function (event) {
  handleKeyPress(event);
};

document.onkeyup = function (event) {
  handleKeyRelease(event);
};

function handleKeyPress(event) {
  clearTimeout(boredTimer);
  boredTimer = null;

  switch (event.key.toLowerCase()) {
    case "w":
      console.log("You pressed the W key - Move forward");
      publishTwist(0.2, 0.3, 0.4);
      playGif("happy");
      break;
    case "a":
      console.log("You pressed the A key - Move left");
      publishTwist(0.2, 0.3, 0.4, -0.2, -0.3, -0.4);
      playGif("default");
      break;
    case "s":
      console.log("You pressed the S key - Move backward");
      publishTwist(-0.2, -0.3, -0.4, 0.0, 0.0, 0.0);
      playGif("angry");
      break;
    case "d":
      console.log("You pressed the D key - Move right");
      publishTwist(0.2, 0.3, 0.4, 0.2, 0.3, 0.4);
      playGif("default");
      break;
    case "x":
      console.log("You stopped the robot");
      publishTwist(0, 0, 0, 0, 0, 0);
      playGif("angry");
      break;
    case "shift":
      console.log(
        "Shift key pressed - Increase speed and play excited animation"
      );
      if (event.key.toLowerCase() === "shift") {
        console.log("Shift key pressed - Speed increased");
        shiftKeyPressed = true;
        playGif("excited");
      }
      break;
  }
}

function publishTwist(
  linearX,
  linearY,
  linearZ,
  angularX = 0,
  angularY = 0,
  angularZ = 0
) {
  var twist = new ROSLIB.Message({
    linear: {
      x: linearX * (shiftKeyPressed ? speed + speedIncrement : speed),
      y: linearY * (shiftKeyPressed ? speed + speedIncrement : speed),
      z: linearZ * (shiftKeyPressed ? speed + speedIncrement : speed),
    },
    angular: {
      x: angularX * (shiftKeyPressed ? speed + speedIncrement : speed),
      y: angularY * (shiftKeyPressed ? speed + speedIncrement : speed),
      z: angularZ * (shiftKeyPressed ? speed + speedIncrement : speed),
    },
  });

  cmdVel.publish(twist);

  isMoving =
    linearX !== 0 ||
    linearY !== 0 ||
    linearZ !== 0 ||
    angularX !== 0 ||
    angularY !== 0 ||
    angularZ !== 0;

  if (!isMoving) {
    boredTimer = setTimeout(function () {
      console.log("MR_Robot is bored");
      playGif("bored");
    }, 10000);
  }
}

function handleKeyRelease(event) {
  if (event.key.toLowerCase() === "shift") {
    console.log("Shift key released - Speed reset to default");
    shiftKeyPressed = false;
    playGif("excited");
  }
}

function playGif(mood = "happy") {
  var animation = document.getElementById("animation");

  switch (mood) {
    case "happy":
      animation.src = "../src/images/Animsx/happy eye anim.mp4";
      break;
    case "angry":
      animation.src = "../src/images/Animsx/angry.mp4";
      break;
    case "bored":
      animation.src = "../src/images/Animsx/bored.mp4";
      break;
    case "excited":
      animation.src = "../src/images/Animsx/excited.mp4";
      break;
    case "dead":
      animation.src = "../src/images/Animsx/ded.mp4";
      break;
    default:
      animation.src = "../src/images/Animsx/default.mp4";
  }
}

ros.on("connection", function () {
  console.log("Connected to ROS!");
});

ros.on("error", function (error) {
  console.error("Error connecting to ROS:", error);
});

ros.on("close", function () {
  console.log("Connection to ROS closed.");
});
