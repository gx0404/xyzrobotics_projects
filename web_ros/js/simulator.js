/**
 * Setup all visualization elements when the page is loaded.
 */
const stats = new Stats();
const viewerId = 'RobotSimulator';
var viewer;
var rosSocket;

function animate() {
  //更新性能插件
  stats.update();
  requestAnimationFrame(animate);
}

function init() {

  // 获取IP地址
  let rosHostInput = document.getElementById("server_host");
  let host = rosHostInput.value;

  // 获取端口地址
  let rosPortInput = document.getElementById("server_port");
  let port = rosPortInput.value;

  // Connect to ROS.
  rosSocket = new ROSLIB.Ros({
    url : `ws://${host}:${port}`
  });

  // Create the main viewer.
  viewer = new ROS3D.Viewer({
    divID : viewerId,
    width : 800,
    height : 600,
    antialias : true,
    background: '#dbdbdb',
    cameraPose: {
      "x": 5,
      "y": 5,
      "z": 5
    }
  });

  // Add a grid.
  viewer.addObject(new ROS3D.Grid());

  // Setup a client to listen to TFs.
  var tfClient = new ROSLIB.TFClient({
    ros : rosSocket,
    angularThres : 0.01,
    transThres : 0.01,
    rate : 30.0,
    fixedFrame: '/map'
  });

  // Setup the URDF client.
  var urdfClient = new ROS3D.UrdfClient({
    ros : rosSocket,
    tfClient : tfClient,
    // path : 'http://resources.robotwebtools.org/',
    path: `http://${host}:7002/static`,
    rootObject : viewer.scene,
    loader : ROS3D.COLLADA_LOADER_2
  });
  
  // 加载real_planning_environment_marker_topic
  var markerArrayClient = new ROS3D.MarkerArrayClient({
    ros: rosSocket,
    tfClient: tfClient,
    camera: viewer.camera,
    rootObject: viewer.scene,
    topic: '/real_planning_environment_marker_topic',
  });
  
  // 加载 booked_items_marker_topic
  var markerArrayClient = new ROS3D.MarkerArrayClient({
    ros: rosSocket,
    tfClient: tfClient,
    camera: viewer.camera,
    rootObject: viewer.scene,
    topic: '/booked_items_marker_topic',
  });

  // 加载 tool_marker_robot_topic
  var markerArrayClient = new ROS3D.MarkerArrayClient({
    ros: rosSocket,
    tfClient: tfClient,
    camera: viewer.camera,
    rootObject: viewer.scene,
    topic: '/tool_marker_robot_topic',
  });
}


function run() {
  init();
  // 加载性能监控插件stats
  document.getElementById(viewerId).appendChild(stats.dom);
  animate();
}