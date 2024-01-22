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

  document
    .getElementsByClassName("connection-button")[0]
    .addEventListener("click", function () {
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
    switch (event.key.toLowerCase()) {
      case "w":
        console.log("You pressed the W key - Move forward");
        publishTwist(0.2, 0.3, 0.4);
        break;
      case "a":
        console.log("You pressed the A key - Move left");
        publishTwist(0.2, 0.3, 0.4, -0.2, -0.3, -0.4);
        break;
      case "s":
        console.log("You pressed the S key - Move backward");
        publishTwist(-0.2, -0.3, -0.4, 0.0, 0.0, 0.0);
        break;
      case "d":
        console.log("You pressed the D key - Move right");
        publishTwist(0.2, 0.3, 0.4, 0.2, 0.3, 0.4);
        break;
      case "x":
        console.log("You stopped the robot");
        publishTwist(0, 0, 0, 0, 0, 0);
        break;
      case "shift":
        console.log("Shift key pressed - Increase speed");
        if (event.key.toLowerCase() === "shift") {
          console.log("Shift key pressed - Speed increased");
          shiftKeyPressed = true;
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

    updateSpeedometerPinRotation(twist.linear.x);

    cmdVel.publish(twist);
  }

  function updateSpeedometerPinRotation(speedValue) {
    var maxSpeed = 1.5;

    var rotationAngle = (speedValue / maxSpeed) * 180 - 90;

    var speedometerPin = document.querySelector(".speedometer-pin");

    speedometerPin.style.transform =
    `translateX(-50%) rotate(${rotationAngle}deg)`
  }

  function handleKeyRelease(event) {
    if (event.key.toLowerCase() === "shift") {
      console.log("Shift key released - Speed reset to default");
      shiftKeyPressed = false;
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