
function init() {
    var ros = new ROSLIB.Ros({
      url: "ws://localhost:9090",
    });
  
    var viewer = new ROS2D.Viewer({
      divID: "smallmap",
      width: 200,
      height: 200,
    });
  
    console.log(viewer);
  
    var gridClient = new ROS2D.OccupancyGridClient({
      ros: ros,
      rootObject: viewer.scene,
    });
  
    // Wait for the gridClient to be fully initialized before scaling
  
    gridClient.on("change", function () {
      setTimeout(function () {
        viewer.scene.scaleX = 55.0; 
        viewer.scene.scaleY = 55.0; 
      }, 100);
    });
    // Connect to ROS after setting up the gridClient
    ros.on("connection", function () {
      // This is where you might want to subscribe to other topics or perform other operations
      console.log("Connected to ROS");
    });
  
    ros.on("error", function (error) {
      console.error("Error connecting to ROS: ", error);
    });
  
    ros.on("close", function () {
      console.log("Connection to ROS closed");
    });
  
    // var nav = new NAV2D.OccupancyGridClientNav({
    //   ros: ros,
    //   topic: "map",
    //   rootObject: viewer.scene,
    //   viewer: viewer,
    //   serverName: "/move_base_simple/goal",
    // });
    function init() {
      // Existing code...
  
      document.addEventListener("mousedown", function () {
        console.log("Global mousedown event triggered");
      });
    }
    var zoomView = new ROS2D.ZoomView({
      rootObject: viewer.scene,
    });
  }
  