#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import tf_transformations
import tf2_ros
from tf2_ros import TransformException
import numpy as np
import pygame
import sys
import traceback
import threading # Added to spin ROS in the background

class HmiNode(Node):
    def __init__(self):
        super().__init__("hmi_node")

        # Pygame Initialization (Will be driven by the main thread loop)
        pygame.init()
        self.screen_ = pygame.display.set_mode((1200, 700))
        pygame.display.set_caption('Prestobot HMI - Custom Button Layout with Hallways')

        self.WHITE_ = (255, 255, 255)
        self.BLACK_ = (0, 0, 0)
        self.BLUE_ = (173, 216, 230)
        self.GREEN_ = (144, 238, 144)
        self.GRAY_ = (200, 200, 200)
        self.ORANGE_ = (255, 178, 102)
        self.font1_ = pygame.font.SysFont('sans', 25)
        self.font_small_ = pygame.font.SysFont('sans', 20)

        # Navigation Setup
        self.navigator_ = BasicNavigator()
        self.navigator_.waitUntilNav2Active()

        # Publishes the exact goal pose sent to Nav2, so external nodes
        # (e.g. nav_error_monitor) can know what the robot was asked to reach.
        self.goal_request_pub_ = self.create_publisher(PoseStamped, 'hmi/requested_goal', 10)

        # TF: robot's real position, to draw the live red dot on the HMI schematic
        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)
        self.robot_world_pos_ = None  # (x, y) in meters, frame 'map'; None until first TF
        self.create_timer(0.1, self.update_robot_pose)  # 10 Hz

        # Label of the room button that is the current target (draws a red marker on it)
        self.active_goal_button_ = None

        # Flow: pick a room (pending) -> Confirm (close door, start moving)
        # -> arrived -> Open (open door) -> back to idle
        self.state_ = 'idle'  # 'idle' | 'pending_confirm' | 'navigating' | 'arrived'
        self.pending_label_ = None
        self.pending_pose_ = None
        self.info_text_ = "Please select a room."
        # Placed in the empty center area of the schematic (right of the vertical
        # corridor, between the top and bottom room rows).
        self.info_box_rect_ = pygame.Rect(450, 270, 650, 90)
        self.action_button_rect_ = pygame.Rect(450, 390, 300, 70)
        
        self.hall_definitions_ = {
            1: set(list(range(1, 15)) + list(range(34, 42))), 
            2: set(list(range(15, 23)) + list(range(42, 48))), 
            3: set(list(range(23, 34)))
        }
        self.intermediate_poses_ = {
            'hall_1_2': self.create_pose_stamped(8.0, 0.0, 0.0), 
            'hall_2_3': self.create_pose_stamped(8.0, 40.0, 0.0)
        }
        self.current_hall_ = 1
        self.room_coordinates = [(65.3, 0.0, -1.57), (58.4, 0.0, -1.57), (55.3, 0.0, -1.57), (48.4, 0.0, -1.57), (45.3, 0.0, -1.57), (38.4, 0.0, -1.57), (35.3, 0.0, -1.57), (28.4, 0.0, -1.57), (25.3, 0.0, -1.57), (18.4, 0.0, -1.57), (15.3, 0.0, -1.57), (8.4, 0.0, -1.57), (5.6, 0.0, -1.57), (8.0, 2.4, 3.14), (8.0, 9.6, 3.14), (8.0, 12.4, 3.14), (8.0, 19.6, 3.14), (8.0, 22.4, 3.14), (8.0, 29.6, 3.14), (8.0, 32.4, 3.14), (8.0, 39.6, 3.14), (65.3, 40.0, -1.57), (58.4, 40.0, -1.57), (55.3, 40.0, -1.57), (48.4, 40.0, -1.57), (45.3, 40.0, -1.57), (38.4, 40.0, -1.57), (35.3, 40.0, -1.57), (28.4, 40.0, -1.57), (25.3, 40.0, -1.57), (18.4, 40.0, -1.57), (15.3, 40.0, -1.57), (30.3, 0.0, 1.57), (33.4, 0.0, 1.57), (40.3, 0.0, 1.57), (43.4, 0.0, 1.57), (50.3, 0.0, 1.57), (53.4, 0.0, 1.57), (60.3, 0.0, 1.57), (63.4, 0.0, 1.57), (8.0, 4.6, 0.0), (8.0, 7.4, 0.0), (8.0, 14.6, 0.0), (8.0, 17.4, 0.0), (8.0, 24.6, 0.0), (8.0, 27.4, 0.0)]
        self.room_button_positions = [(1, (1080, 620)), (2, (1000, 620)), (3, (920, 620)), (4, (840, 620)), (5, (760, 620)), (6, (680, 620)), (7, (600, 620)), (8, (520, 620)), (9, (440, 620)), (10, (360, 620)), (11, (280, 620)), (12, (200, 620)), (14, (120, 620)), (15, (120, 500)), (16, (120, 440)), (17, (120, 380)), (18, (120, 320)), (19, (120, 260)), (20, (120, 200)), (21, (120, 140)), (22, (120, 80)), (23, (1080, 140)), (24, (1000, 140)), (25, (920, 140)), (26, (840, 140)), (27, (760, 140)), (28, (680, 140)), (29, (600, 140)), (30, (520, 140)), (31, (440, 140)), (32, (360, 140)), (33, (280, 140)), (34, (520, 500)), (35, (600, 500)), (36, (680, 500)), (37, (760, 500)), (38, (840, 500)), (39, (920, 500)), (40, (1000, 500)), (41, (1080, 500)), (42, (280, 500)), (43, (280, 440)), (44, (280, 380)), (45, (280, 320)), (46, (280, 260)), (47, (280, 200))]

        self.buttons_ = []
        self.generate_buttons()
        self.compute_world_to_screen_transform()
        self.get_logger().info("HMI Node structural initialization finished.")

    def generate_buttons(self):
        home_pose = self.create_pose_stamped(0.0, 0.0, 0.0)
        home_text = self.font1_.render('Home', True, self.BLACK_)
        home_rect = pygame.Rect(50, 555, 120, 60)
        self.buttons_.append({'text': home_text, 'rect': home_rect, 'pose': home_pose, 'label': 'Home', 'color': self.GREEN_})
        button_width, button_height = 80, 50
        room_coords_iter = iter(self.room_coordinates)
        for room_label, pos in self.room_button_positions:
            x_pos, y_pos = pos
            coords = next(room_coords_iter)
            text_surface = self.font1_.render(str(room_label), True, self.BLACK_)
            button_rect = pygame.Rect(x_pos, y_pos, button_width, button_height)
            goal_pose = self.create_pose_stamped(*coords)
            self.buttons_.append({'text': text_surface, 'rect': button_rect, 'pose': goal_pose, 'label': f'Room {room_label}', 'color': self.BLUE_})

    def compute_world_to_screen_transform(self):
        """
        Estimate an affine transform (meters in the 'map' frame -> screen pixels)
        via least-squares, using the real coordinate + screen position of every
        button already created (Home + all rooms) as correspondence points.
        No need to redraw the schematic to true scale, just needs the buttons
        to already be reasonably placed on screen.
        """
        world_pts = np.array([
            [b['pose'].pose.position.x, b['pose'].pose.position.y, 1.0]
            for b in self.buttons_
        ])
        screen_x = np.array([b['rect'].center[0] for b in self.buttons_])
        screen_y = np.array([b['rect'].center[1] for b in self.buttons_])
        self.coeff_x_, *_ = np.linalg.lstsq(world_pts, screen_x, rcond=None)
        self.coeff_y_, *_ = np.linalg.lstsq(world_pts, screen_y, rcond=None)

    def world_to_screen(self, wx, wy):
        ax, bx, cx = self.coeff_x_
        ay, by, cy = self.coeff_y_
        return int(ax * wx + bx * wy + cx), int(ay * wx + by * wy + cy)

    def update_robot_pose(self):
        """Runs on the ROS thread (timer callback); just records the latest pose."""
        try:
            tf = self.tf_buffer_.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
            self.robot_world_pos_ = (tf.transform.translation.x, tf.transform.translation.y)
        except TransformException:
            pass  # no TF yet (e.g. just started) -> keep the last known point, skip this frame

    def get_hall_for_room(self, room_label):
        if room_label == 'Home': return 1
        try:
            room_number = int(room_label.split(' ')[1])
            for hall, rooms in self.hall_definitions_.items():
                if room_number in rooms: return hall
        except: return None
        return None

    def create_pose_stamped(self, position_x, position_y, rotation_z):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = position_x
        goal_pose.pose.position.y = position_y
        goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = q_x, q_y, q_z, q_w
        return goal_pose

    def handle_navigation_request(self, destination_label, destination_pose):
        destination_hall = self.get_hall_for_room(destination_label)
        if destination_hall is None: return

        self.get_logger().info(f"Nav request received for: {destination_label}")
        self.active_goal_button_ = destination_label

        # Re-stamp with current time and broadcast so error-monitoring nodes
        # know exactly which pose this run is being judged against.
        destination_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_request_pub_.publish(destination_pose)

        if self.current_hall_ == destination_hall:
            self.navigator_.goToPose(destination_pose)
        else:
            waypoints = []
            if (self.current_hall_, destination_hall) in [(1, 2), (2, 1)]:
                waypoints.append(self.intermediate_poses_['hall_1_2'])
            elif (self.current_hall_, destination_hall) in [(2, 3), (3, 2)]:
                waypoints.append(self.intermediate_poses_['hall_2_3'])
            elif (self.current_hall_, destination_hall) == (1, 3):
                waypoints.extend([self.intermediate_poses_['hall_1_2'], self.intermediate_poses_['hall_2_3']])
            elif (self.current_hall_, destination_hall) == (3, 1):
                waypoints.extend([self.intermediate_poses_['hall_2_3'], self.intermediate_poses_['hall_1_2']])
            
            waypoints.append(destination_pose)
            self.navigator_.followWaypoints(waypoints)
        
        self.current_hall_ = destination_hall

    def select_destination(self, label, pose):
        """Clicking a room button only STORES the pending destination; the robot
        doesn't move yet. Destination can't be changed while navigating or
        while waiting for the door to be opened."""
        if self.state_ not in ('idle', 'pending_confirm'):
            return
        self.pending_label_ = label
        self.pending_pose_ = pose
        self.state_ = 'pending_confirm'
        self.active_goal_button_ = label
        self.info_text_ = f"Selected: {label}. Press 'Confirm' to close the door and start moving."

    def confirm_and_go(self):
        """Pressing Confirm: close the door (status/info text only for now, no
        real actuator yet), then actually issue the navigation command."""
        if self.state_ != 'pending_confirm' or self.pending_label_ is None:
            return
        self.info_text_ = "Door closed."
        self.state_ = 'navigating'
        self.handle_navigation_request(self.pending_label_, self.pending_pose_)

    def open_compartment(self):
        """Pressing Open once arrived, then return to idle so a new room can
        be selected."""
        if self.state_ != 'arrived':
            return
        self.info_text_ = "Door opened."
        self.state_ = 'idle'
        self.pending_label_ = None
        self.pending_pose_ = None
        self.active_goal_button_ = None

    def check_navigation_result(self):
        """Called every frame while 'navigating': non-blocking, isTaskComplete()
        only checks whether the future is already done, it doesn't spin itself."""
        if self.state_ != 'navigating':
            return
        if not self.navigator_.isTaskComplete():
            return
        result = self.navigator_.getResult()
        self.state_ = 'arrived'
        if result == TaskResult.SUCCEEDED:
            self.info_text_ = "Arrived. Press 'Open' to open the door."
        else:
            self.info_text_ = "Failed to reach destination. Press 'Open' to finish."

    def draw_wrapped_text(self, text, rect, font, color):
        """Simple word-wrap: fits text inside rect.width, drawing top-aligned lines."""
        words = text.split(' ')
        lines = []
        current = ''
        for w in words:
            trial = (current + ' ' + w).strip()
            if font.size(trial)[0] <= rect.width - 16:
                current = trial
            else:
                if current:
                    lines.append(current)
                current = w
        if current:
            lines.append(current)
        line_height = font.get_linesize()
        y = rect.y + 8
        for line in lines:
            surf = font.render(line, True, color)
            self.screen_.blit(surf, (rect.x + 8, y))
            y += line_height

    def update_ui(self):
        """ This method runs natively on the Main Thread via a standard while loop """
        self.screen_.fill(self.WHITE_)

        # Check whether we've arrived (non-blocking, just checks if the future is done)
        self.check_navigation_result()

        # Draw hallways
        pygame.draw.line(self.screen_, self.GRAY_, (180, 585), (1160, 585), 50)
        pygame.draw.line(self.screen_, self.GRAY_, (240, 80), (240, 560), 50)
        pygame.draw.line(self.screen_, self.GRAY_, (215, 105), (1160, 105), 50)

        mouse_pos = pygame.mouse.get_pos()
        rooms_enabled = self.state_ == 'idle'
        for button in self.buttons_:
            color = button['color'] if rooms_enabled else self.GRAY_
            pygame.draw.rect(self.screen_, color, button['rect'])
            text_rect = button['text'].get_rect(center=button['rect'].center)
            self.screen_.blit(button['text'], text_rect)
            # Small red marker on the corner of the button that is the current target
            if button['label'] == self.active_goal_button_:
                marker_pos = (button['rect'].right - 8, button['rect'].top + 8)
                pygame.draw.circle(self.screen_, (255, 0, 0), marker_pos, 6)

        # Big red dot showing the robot's real position (TF map -> base_footprint)
        if self.robot_world_pos_ is not None:
            robot_screen_pos = self.world_to_screen(*self.robot_world_pos_)
            pygame.draw.circle(self.screen_, (255, 0, 0), robot_screen_pos, 10)
            pygame.draw.circle(self.screen_, self.BLACK_, robot_screen_pos, 10, width=1)

        # Info box
        pygame.draw.rect(self.screen_, self.WHITE_, self.info_box_rect_)
        pygame.draw.rect(self.screen_, self.BLACK_, self.info_box_rect_, width=2)
        self.draw_wrapped_text(self.info_text_, self.info_box_rect_, self.font_small_, self.BLACK_)

        # Action button: switches role between Confirm / Open depending on state
        action_label = None
        if self.state_ == 'pending_confirm':
            action_label = 'Confirm'
            pygame.draw.rect(self.screen_, self.GREEN_, self.action_button_rect_)
        elif self.state_ == 'arrived':
            action_label = 'Open'
            pygame.draw.rect(self.screen_, self.ORANGE_, self.action_button_rect_)
        if action_label:
            pygame.draw.rect(self.screen_, self.BLACK_, self.action_button_rect_, width=2)
            action_surface = self.font1_.render(action_label, True, self.BLACK_)
            action_text_rect = action_surface.get_rect(center=self.action_button_rect_.center)
            self.screen_.blit(action_surface, action_text_rect)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.MOUSEBUTTONDOWN:
                if self.action_button_rect_.collidepoint(mouse_pos):
                    if self.state_ == 'pending_confirm':
                        self.confirm_and_go()
                    elif self.state_ == 'arrived':
                        self.open_compartment()
                elif rooms_enabled:
                    for button in self.buttons_:
                        if button['rect'].collidepoint(mouse_pos):
                            self.select_destination(button['label'], button['pose'])
                            break

        pygame.display.flip()
        return True

def main(args=None):
    rclpy.init(args=args)
    node = HmiNode()
    
    # 1. MultiThreadedExecutor processes background ROS 2 communications/actions
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    # 2. Spin the ROS executor in a separate background thread
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    
    # 3. Use the Main Thread explicitly for Pygame window execution 
    clock = pygame.time.Clock()
    running = True
    
    try:
        while running and rclpy.ok():
            running = node.update_ui()
            clock.tick(30) # Maintain a crisp, stable 30 FPS UI redraw rate
    except KeyboardInterrupt:
        pass
    except Exception:
        # Previously an error here was swallowed by "finally: sys.exit()", so
        # the HMI would close silently with no clue why. Log the traceback first.
        node.get_logger().error("HMI crashed:\n" + traceback.format_exc())
    finally:
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit()

if __name__ == "__main__":
    main()
