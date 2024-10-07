extends Node3D

var ros_viewport1 = ViewPort.new()
var ros_viewport2 = ViewPort.new()

var viewport1: SubViewport
var viewport2: SubViewport
var camera1: Camera3D
var camera2: Camera3D

func _ready():
	print("Setting up cameras and viewports")
	
	# Set unique node names for each ViewPort
	ros_viewport1.set_node_name("godot_image_node1")
	ros_viewport2.set_node_name("godot_image_node2")
	
	# Get existing SubViewports
	viewport1 = $"../SubViewport"
	viewport2 = $"../SubViewport2"
	
	# Ensure SubViewports are set to update
	viewport1.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	viewport2.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	
	# Get existing cameras
	camera1 = $"../SubViewport/Camera3D"
	camera2 = $"../SubViewport2/Camera3D"
	
	# Ensure cameras are current in their respective SubViewports
	camera1.make_current()
	camera2.make_current()
	
	# Set up ROS nodes
	ros_viewport1.set_topic("camera1_image_topic")
	ros_viewport2.set_topic("camera2_image_topic")
	
	# Set up timer for periodic publishing
	var timer = Timer.new()
	add_child(timer)
	timer.connect("timeout", Callable(self, "_on_timer_timeout"))
	timer.set_wait_time(0.05)  # Publish every 0.05 seconds
	timer.set_one_shot(false)
	timer.start()

func _on_timer_timeout():
	await RenderingServer.frame_post_draw
	
	# Publish camera1 image
	var img1 = viewport1.get_texture().get_image()
	img1.convert(Image.FORMAT_RGB8)
	ros_viewport1.pubImage(img1)
	
	# Publish camera2 image
	var img2 = viewport2.get_texture().get_image()
	img2.convert(Image.FORMAT_RGB8)
	ros_viewport2.pubImage(img2)
	
	# Spin both nodes
	ros_viewport1.spin_some()
	ros_viewport2.spin_some()
	
	print("Published images from both cameras")

# You can add methods to move/rotate cameras if needed
func move_camera1(new_position: Vector3):
	camera1.global_transform.origin = new_position
	print("Moved Camera1 to ", new_position)

func move_camera2(new_position: Vector3):
	camera2.global_transform.origin = new_position
	print("Moved Camera2 to ", new_position)
