
        var src = "";

        function CameraOn() {
            src = "http://localhost:8080/stream?topic=/rrbot/camera1/image_raw";
            document.getElementById("dynamicImage").src = src; // Update the src attribute
            console.log("on");
        }

        function CameraOff() {
            src = "#";
            document.getElementById("dynamicImage").src = src; // Update the src attribute
            console.log("off");
    }