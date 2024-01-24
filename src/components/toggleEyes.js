// Define a function to toggle the class that handles the opening and closing effect
function toggleMood() {
  var eyesBackElement = document.querySelector(".eyes-back");

  // Toggle the 'open' class to handle the opening and closing effect
  eyesBackElement.classList.toggle("open");
}

// Add an event listener to the "eyes-back" element to trigger the toggleMood function when clicked
document.querySelector(".eyes-back").addEventListener("click", toggleMood);
