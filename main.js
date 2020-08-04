// ws://54.147.26.233:9090

var app = new Vue({
    el: '#app',
    // computed values
    computed: {
        ws_address: function () {
            return `${this.rosbridge_address}`
        },
    },
    // storing the state of the page
    data: {
        connected: false,
        ros: null,
        logs: [],
        loading: false,
        topic: null,
        message: null,
        rosbridge_address: 'ws://192.168.0.105:9090',
        menu_title: "Setup",
        position: { x: 0, y: 0, z: 0, },
        service_busy: false,
        service_response: '',
        port: '9090',
        mapViewer: null,
        mapGridClient: null,
        interval: null,
        linear_x_val: 0,
        angular_z_val: 0,
        param_read_val: 0,
        // 3D stuff
        viewer: null,
        tfClient: null,
        urdfClient: null,
        // ROS_Action
        goal: null,
        action: {
            goal: { position: { x: 0, y: 0, z: 0 } },
            feedback: { position: 0, state: 'idle' },
            result: { success: false },
            status: { status: 0, text: '' },
        },
        dragging: false,
        x: 'no',
        y: 'no',
        dragCircleStyle: {
            margin: '0px',
            top: '0px',
            left: '0px',
            display: 'none',
            width: '75px',
            height: '75px',
        },
        // joystick valules
        joystick: {
            vertical: 0,
            horizontal: 0,
        }

    },
    // helper methods to connect to ROS
    methods: {
        connect: function () {
            this.loading = true
            this.ros = new ROSLIB.Ros({
                url: this.ws_address
            })
            this.ros.on('connection', () => {
                this.connected = true
                console.log('Connection to ROSBridge established!')
                let topic = new ROSLIB.Topic({
                    ros: this.ros,
                    name: '/odom',
                    messageType: 'nav_msgs/Odometry'
                })
                topic.subscribe((message) => {
                    this.position = message.pose.pose.position

                })
                this.logs.unshift((new Date()).toTimeString() + ' - Connected!!!')
                this.loading = false
                this.setCamera()
                this.setCamera2()
                this.setup3DViewer()

                //2D Map
                this.mapViewer = new ROS2D.Viewer({
                    divID: 'map',
                    width: 420,
                    height: 360,
                })
                // Setup the map client.
                this.mapGridClient = new ROS2D.OccupancyGridClient({
                    ros: this.ros,
                    rootObject: this.mapViewer.scene,
                    continuous: true,
                })
                // Scale the canvas to fit to the map
                this.mapGridClient.on('change', () => {
                    this.mapViewer.scaleToDimensions(this.mapGridClient.currentGrid.width, this.mapGridClient.currentGrid.height);
                    this.mapViewer.shift(this.mapGridClient.currentGrid.pose.position.x, this.mapGridClient.currentGrid.pose.position.y)
                })


            })
            this.ros.on('error', (error) => {
                this.logs.unshift((new Date()).toTimeString() + ` - Error: ${error}`)
            })
            this.ros.on('close', () => {
                this.connected = false
                console.log('Connection to ROSBridge was closed!')
                this.logs.unshift((new Date()).toTimeString() + ' - Disconnected!')
                this.loading = false
                document.getElementById('divCamera').innerHTML = ''
                document.getElementById('map').innerHTML = ''
                this.unset3DViewer()
            })
        },
        disconnect: function () {
            this.ros.close()
        },
        setTopic: function () {
            this.topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })
        },
        forward: function () {
            this.message = new ROSLIB.Message({
                linear: { x: 0.5, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: 0, },
            })
            this.setTopic()
            this.topic.publish(this.message)
        },
        stop: function () {
            this.message = new ROSLIB.Message({
                linear: { x: 0, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: 0, },
            })
            this.setTopic()
            this.topic.publish(this.message)
        },
        backward: function () {
            this.message = new ROSLIB.Message({
                linear: { x: -0.5, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: 0, },
            })
            this.setTopic()
            this.topic.publish(this.message)
        },
        turnLeft: function () {
            this.message = new ROSLIB.Message({
                linear: { x: 0.5, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: 0.5, },
            })
            this.setTopic()
            this.topic.publish(this.message)
        },
        turnRight: function () {
            this.message = new ROSLIB.Message({
                linear: { x: 0.5, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: -0.5, },
            })
            this.setTopic()
            this.topic.publish(this.message)
        },
        setCamera: function () {
            let without_wss = this.rosbridge_address.split('ws://')[1]
            console.log(without_wss)
            let domain = without_wss.split('/')[0]
            console.log(domain)
            let host = '192.168.0.105:11315'
            let viewer = new MJPEGCANVAS.Viewer({
                divID: 'divCamera',
                host: host,
                width: 320,
                height: 240,
                topic: '/raspicam_node/image&type=ros_compressed',
                ssl: false,
                image_transport: "compressed",
            })
        },
        setCamera2: function () {
            let without_wss = this.rosbridge_address.split('ws://')[1]
            console.log(without_wss)
            let domain = without_wss.split('/')[0]
            console.log(domain)
            let host = domain + '/cameras'
            let viewer = new MJPEGCANVAS.Viewer({
                divID: 'divCamera2',
                host: host,
                width: 320,
                height: 240,
                topic: '/raspicam_node/image/compressed',
                ssl: false,

            })
        },
        set_param: function () {
            // set as busy
            service_busy = true

            let linear_param = new ROSLIB.Param({
                ros: this.ros,
                name: 'web_param/linear_x'
            })

            let angular_param = new ROSLIB.Param({
                ros: this.ros,
                name: 'web_param/angular_z'
            })
            linear_param.set(this.linear_x_val)
            angular_param.set(this.angular_z_val)

            // set as not busy
            service_busy = false
        },
        // read_param: function () {
        //     // set as busy
        //     service_busy = true

        //     let web_param = new ROSLIB.Param({
        //         ros: this.ros,
        //         name: 'web_param'
        //     })

        //     web_param.get((value) => {
        //         // set as not busy
        //         service_busy = false
        //         this.param_read_val = value
        //     }, (err) => {
        //         // set as not busy
        //         service_busy = false
        //     })

        // },
        stop_param: function () {
            // set as busy
            service_busy = true

            let linear_param = new ROSLIB.Param({
                ros: this.ros,
                name: 'web_param/linear_x'
            })

            let angular_param = new ROSLIB.Param({
                ros: this.ros,
                name: 'web_param/angular_z'
            })
            this.linear_x_val = 0
            this.angular_z_val = 0
            linear_param.set(this.linear_x_val)
            angular_param.set(this.angular_z_val)

            // set as not busy
            service_busy = false
        },

        setup3DViewer() {
            this.viewer = new ROS3D.Viewer({
                divID: 'div3DViewer',
                width: 400,
                height: 300,
                antialias: true,
                fixedFrame: 'odom'
            })
            // Add a grid.
            this.viewer.addObject(new ROS3D.Grid({
                color: '#0181c4',
                cellSize: 0.5,
                num_cells: 20
            }))
            // Setup a client to listen to TFs.
            this.tfClient = new ROSLIB.TFClient({
                ros: this.ros,
                angularThres: 0.01,
                transThres: 0.01,
                rate: 10.0
            })
            // Setup the URDF client.
            this.urdfClient = new ROS3D.UrdfClient({
                ros: this.ros,
                param: 'robot_description',
                tfClient: this.tfClient,
                path: window.location.href,
                rootObject: this.viewer.scene,
                loader: ROS3D.COLLADA_LOADER_2
            })
        },
        unset3DViewer() {
            document.getElementById('div3DViewer').innerHTML = ''
        },

        //ROS_Action
        sendGoal: function () {
            let actionClient = new ROSLIB.ActionClient({
                ros: this.ros,
                serverName: '/turtlebot2_action_service_as',
                actionName: 'course_web_dev_ros/WaypointActionAction'
            })

            this.goal = new ROSLIB.Goal({
                actionClient: actionClient,
                goalMessage: {
                    position: this.action.goal.position
                }

            })

            this.goal.on('status', (status) => {
                this.action.status = status
            })

            this.goal.on('feedback', (feedback) => {
                this.action.feedback = feedback
            })

            this.goal.on('result', (result) => {
                this.action.result = result
            })

            this.goal.send()
        },
        cancelGoal: function () {
            this.goal.cancel()
        },

        sendCommand: function () {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })

            if (this.joystick.vertical >= 0) {
                let message = new ROSLIB.Message({
                    linear: { x: this.joystick.vertical, y: 0, z: 0, },
                    angular: { x: 0, y: 0, z: -this.joystick.horizontal },
                })
                topic.publish(message)
            }
            else {
                let message = new ROSLIB.Message({
                    linear: { x: this.joystick.vertical, y: 0, z: 0, },
                    angular: { x: 0, y: 0, z: this.joystick.horizontal },
                })
                topic.publish(message)
            }

        },
        startDrag() {
            this.dragging = true
            this.x = this.y = 0
        },
        stopDrag() {
            this.dragging = false
            this.x = this.y = 'no'
            this.dragCircleStyle.display = 'none'
            this.resetJoystickVals()
        },
        doDrag(event) {
            if (this.dragging) {
                this.x = event.offsetX
                this.y = event.offsetY
                let ref = document.getElementById('dragstartzone')
                this.dragCircleStyle.display = 'inline-block'

                let minTop = ref.offsetTop - parseInt(this.dragCircleStyle.height) / 2
                let maxTop = minTop + 200
                let top = this.y + minTop
                this.dragCircleStyle.top = `${top}px`

                let minLeft = ref.offsetLeft - parseInt(this.dragCircleStyle.width) / 2
                let maxLeft = minLeft + 200
                let left = this.x + minLeft
                this.dragCircleStyle.left = `${left}px`

                this.setJoystickVals()
            }
        },
        setJoystickVals() {
            this.joystick.vertical = -1 * ((this.y / 200) - 0.5)
            this.joystick.horizontal = +1 * ((this.x / 200) - 0.5)
            this.sendCommand()
        },
        resetJoystickVals() {
            this.joystick.vertical = 0
            this.joystick.horizontal = 0
            this.sendCommand()
        },
        home_position() {
            this.action.goal.position.x = 0
            this.action.goal.position.y = 0
        },
        a_position() {
            this.action.goal.position.x = 3
            this.action.goal.position.y = 3
        },
        b_position() {
            this.action.goal.position.x = 3
            this.action.goal.position.y = -3
        },
        c_position() {
            this.action.goal.position.x = -3
            this.action.goal.position.y = 3
        },
        d_position() {
            this.action.goal.position.x = -3
            this.action.goal.position.y = -3
        },

    },
    mounted() {
        window.addEventListener('mouseup', this.stopDrag)

    },
})

