let scene, camera, renderer, cube;

function parentWidth(elem) {
  return elem.parentElement.clientWidth;
}

function parentHeight(elem) {
  return elem.parentElement.clientHeight;
}

function init3D() {
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0xffffff);

  camera = new THREE.PerspectiveCamera(
    75,
    parentWidth(document.getElementById("3Dcube")) /
      parentHeight(document.getElementById("3Dcube")),
    0.1,
    1000
  );

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(
    parentWidth(document.getElementById("3Dcube")),
    parentHeight(document.getElementById("3Dcube"))
  );

  document.getElementById("3Dcube").appendChild(renderer.domElement);

  // Create a geometry
  const geometry = new THREE.BoxGeometry(5, 1, 4);

  // Materials of each face with distinct colors
  var cubeMaterials = [
    new THREE.MeshBasicMaterial({ color: 0xff0000 }), // Right face (red)
    new THREE.MeshBasicMaterial({ color: 0x00ff00 }), // Left face (green)
    new THREE.MeshBasicMaterial({ color: 0x0000ff }), // Top face (blue)
    new THREE.MeshBasicMaterial({ color: 0xffff00 }), // Bottom face (yellow)
    new THREE.MeshBasicMaterial({ color: 0xff00ff }), // Front face (magenta)
    new THREE.MeshBasicMaterial({ color: 0x00ffff })  // Back face (cyan)
  ];

  const material = new THREE.MeshFaceMaterial(cubeMaterials);

  cube = new THREE.Mesh(geometry, material);
  scene.add(cube);
  camera.position.z = 5;
  renderer.render(scene, camera);
  
  // Add axis lines (X, Y, Z)
  const axisHelper = new THREE.AxesHelper(5);
  scene.add(axisHelper);
}

// Resize the 3D object when the browser window changes size
function onWindowResize() {
  camera.aspect = parentWidth(document.getElementById("3Dcube")) / parentHeight(document.getElementById("3Dcube"));
  camera.updateProjectionMatrix();
  renderer.setSize(
    parentWidth(document.getElementById("3Dcube")),
    parentHeight(document.getElementById("3Dcube"))
  );
}

window.addEventListener('resize', onWindowResize, false);

// Create the 3D representation
init3D();

// Create events for the sensor readings
if (!!window.EventSource) {
  var source = new EventSource('/events');

  source.addEventListener('open', function(e) {
    console.log("Events Connected");
  }, false);

  source.addEventListener('error', function(e) {
    if (e.target.readyState != EventSource.OPEN) {
      console.log("Events Disconnected");
    }
  }, false);

  source.addEventListener('imu_readings', function(e) {
    //console.log("gyro_readings", e.data);
    var obj = JSON.parse(e.data);
    document.getElementById("gyro_roll").innerHTML = obj.angle_roll;
    document.getElementById("gyro_pitch").innerHTML = obj.angle_pitch;
    document.getElementById("gyro_yaw").innerHTML = obj.gyro_yaw_input;

    document.getElementById("acc_x").innerHTML = obj.ax_mps2;
    document.getElementById("acc_y").innerHTML = obj.ay_mps2;
    document.getElementById("acc_z").innerHTML = obj.az_mps2;

    // Change cube rotation after receiving the readinds
  cube.rotation.x = THREE.Math.degToRad(obj.gyro_pitch);   // Pitch
  cube.rotation.y = THREE.Math.degToRad(obj.gyro_yaw);     // Yaw
  cube.rotation.z = THREE.Math.degToRad(obj.gyro_roll);    // Roll
  
    renderer.render(scene, camera);
  }, false);
}

