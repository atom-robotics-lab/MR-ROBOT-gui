function submit() {
  var formData = {};

  function rosIP() {
    var rosInput = document.getElementById("ros");
    formData.rosServerIP = rosInput.value;
    console.log("ros server ip", formData.rosServerIP);
  }

  function localhostIP() {
    var localhostInput = document.getElementById("localhost");
    formData.localhostIP = localhostInput.value;
    console.log("localhost server ip", formData.localhostIP);
  }

  function videoIP() {
    var videoInput = document.getElementById("videoserver");
    formData.videoServerIP = videoInput.value;
    console.log("video server ip", formData.videoServerIP);
  }

  rosIP();
  localhostIP();
  videoIP();

  // Store the form data in local storage
  localStorage.setItem("formData", JSON.stringify(formData));

  // Redirect to index.html
  window.location.href = "./main.html";
}

// Add this function to retrieve the form data from local storage on index.html
function displayFormData() {
  var formDataString = localStorage.getItem("formData");
  if (formDataString) {
    var formData = JSON.parse(formDataString);
    console.log("Form Data on index.html:", formData);

    // You can now use the formData object to display the values on index.html
    // For example, update the content of elements with specific IDs on index.html
    document.getElementById("rosValue").textContent =
      "ROS SERVER IP : " + "" + formData.rosServerIP;
    document.getElementById("localhostValue").textContent =
      "localhost IPv4 : " + "" + formData.localhostIP;
    document.getElementById("videoServerValue").textContent =
      "VideoServer IP : " + "" + formData.videoServerIP;
  }
}
window.onload = function () {
  displayFormData();
};
