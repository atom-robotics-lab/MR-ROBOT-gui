var MapSrc = "";
function FullMapView() {
  var MapviewOpen = document.getElementById("MapView");
  if (MapviewOpen) {
    MapviewOpen.style.visibility = "visible";
  }
}
function MapViewClose() {
  var MapviewOpen = document.getElementById("MapView");
  if ((MapviewOpen.style.visibility = "visible")) {
    MapviewOpen.style.visibility = "hidden";
  }
}
