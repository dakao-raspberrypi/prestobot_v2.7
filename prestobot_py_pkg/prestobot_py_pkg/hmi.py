#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import pygame

class HmiNode(Node):
    def __init__(self):
        super().__init__("hmi_node")

        # Pygame Initialization
        pygame.init()
        self.screen_ = pygame.display.set_mode((1200, 700))
        pygame.display.set_caption('Prestobot HMI - Custom Button Layout with Hallways')

        # Colors and Font
        self.WHITE_ = (255, 255, 255)
        self.BLACK_ = (0, 0, 0)
        self.BLUE_ = (173, 216, 230)
        self.GREEN_ = (144, 238, 144)
        self.GRAY_ = (200, 200, 200) # Color for hallways
        self.font1_ = pygame.font.SysFont('sans', 25)

        # Navigation Setup
        self.navigator_ = BasicNavigator()
        self.navigator_.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active. Ready to send goals.")

        # Hall Definitions
        self.hall_definitions_ = {
            1: set(list(range(1, 15)) + list(range(34, 42))),
            2: set(list(range(15, 23)) + list(range(42, 48))),
            3: set(list(range(23, 34)))
        }

        # Intermediate Waypoints
        intermediate_coord_1 = (8.0, 0.0, 0.0)
        intermediate_coord_2 = (8.0, 40.0, 0.0)
        self.intermediate_poses_ = {
            'hall_1_2': self.create_pose_stamped(*intermediate_coord_1),
            'hall_2_3': self.create_pose_stamped(*intermediate_coord_2)
        }
        self.current_hall_ = 1

        # Room Coordinates (Unchanged)
        self.room_coordinates = [
            (65.3, 0.0, -1.57), #1
            (58.4, 0.0, -1.57), #2
            (55.3, 0.0, -1.57), #3
            (48.4, 0.0, -1.57), #4
            (45.3, 0.0, -1.57), #5
            (38.4, 0.0, -1.57), #6
            (35.3, 0.0, -1.57), #7
            (28.4, 0.0, -1.57), #8
            (25.3, 0.0, -1.57), #9
            (18.4, 0.0, -1.57), #10
            (15.3, 0.0, -1.57), #11
            (8.4, 0.0, -1.57),  #12
            # Skip Room 13
            (5.6, 0.0, -1.57), #14
            (8.0, 2.4, 3.14), #15
            (8.0, 9.6, 3.14), #16
            (8.0, 12.4, 3.14), #17
            (8.0, 19.6, 3.14), #18
            (8.0, 22.4, 3.14), #19
            (8.0, 29.6, 3.14), #20
            (8.0, 32.4, 3.14), #21
            (8.0, 39.6, 3.14), #22
            (65.3, 40.0, -1.57), #23
            (58.4, 40.0, -1.57), #24
            (55.3, 40.0, -1.57), #25
            (48.4, 40.0, -1.57), #26
            (45.3, 40.0, -1.57), #27
            (38.4, 40.0, -1.57), #28
            (35.3, 40.0, -1.57), #29
            (28.4, 40.0, -1.57), #30
            (25.3, 40.0, -1.57), #31
            (18.4, 40.0, -1.57), #32
            (15.3, 40.0, -1.57), #33
            (30.3, 0.0, 1.57), #34
            (33.4, 0.0, 1.57), #35
            (40.3, 0.0, 1.57), #36
            (43.4, 0.0, 1.57), #37
            (50.3, 0.0, 1.57), #38
            (53.4, 0.0, 1.57), #39
            (60.3, 0.0, 1.57), #40
            (63.4, 0.0, 1.57), #41
            (8.0, 4.6, 0.0),  #42
            (8.0, 7.4, 0.0),  #43
            (8.0, 14.6, 0.0), #44
            (8.0, 17.4, 0.0), #45
            (8.0, 24.6, 0.0), #46
            (8.0, 27.4, 0.0)  #47
        ]

        # Custom Button Positions
        self.room_button_positions = [
            (1, (1080, 620)), (2, (1000, 620)), (3, (920, 620)),
            (4, (840, 620)), (5, (760, 620)), (6, (680, 620)),
            (7, (600, 620)), (8, (520, 620)), (9, (440, 620)),
            (10, (360, 620)), (11, (280, 620)), (12, (200, 620)),
            (14, (120, 620)), (15, (120, 500)), (16, (120, 440)),
            (17, (120, 380)), (18, (120, 320)), (19, (120, 260)),
            (20, (120, 200)), (21, (120, 140)), (22, (120, 80)),
            (23, (1080, 140)), (24, (1000, 140)), (25, (920, 140)),
            (26, (840, 140)), (27, (760, 140)), (28, (680, 140)),
            (29, (600, 140)), (30, (520, 140)), (31, (440, 140)),
            (32, (360, 140)), (33, (280, 140)), (34, (520, 500)),
            (35, (600, 500)), (36, (680, 500)), (37, (760, 500)),
            (38, (840, 500)), (39, (920, 500)), (40, (1000, 500)),
            (41, (1080, 500)), (42, (280, 500)), (43, (280, 440)),
            (44, (280, 380)), (45, (280, 320)), (46, (280, 260)),
            (47, (280, 200))
        ]

        self.buttons_ = []
        self.generate_buttons()
        self.timer_ = self.create_timer(0.02, self.update)
        self.get_logger().info("HMI with custom button layout has been started.")

    def generate_buttons(self):
        # Create and add the Home button
        home_pose = self.create_pose_stamped(0.0, 0.0, 0.0)
        home_text = self.font1_.render('Home', True, self.BLACK_)
        home_rect = pygame.Rect(50, 555, 120, 60)
        self.buttons_.append({
            'text': home_text, 'rect': home_rect, 'pose': home_pose,
            'label': 'Home', 'color': self.GREEN_
        })

        # Create and add the Room buttons with custom positions
        button_width, button_height = 80, 50
        room_coords_iter = iter(self.room_coordinates)
        for room_label, pos in self.room_button_positions:
            x_pos, y_pos = pos
            coords = next(room_coords_iter)
            text_surface = self.font1_.render(str(room_label), True, self.BLACK_)
            button_rect = pygame.Rect(x_pos, y_pos, button_width, button_height)
            goal_pose = self.create_pose_stamped(*coords)
            self.buttons_.append({
                'text': text_surface, 'rect': button_rect, 'pose': goal_pose,
                'label': f'Room {room_label}', 'color': self.BLUE_
            })

    def get_hall_for_room(self, room_label):
        if room_label == 'Home':
            return 1
        try:
            room_number = int(room_label.split(' ')[1])
            for hall, rooms in self.hall_definitions_.items():
                if room_number in rooms:
                    return hall
        except (ValueError, IndexError):
            return None
        return None

    def handle_navigation_request(self, destination_label, destination_pose):
        destination_hall = self.get_hall_for_room(destination_label)
        if destination_hall is None:
            self.get_logger().error(f"Could not determine hall for '{destination_label}'")
            return

        self.get_logger().info(f"Request: Current Hall={self.current_hall_}, Destination='{destination_label}' in Hall={destination_hall}")

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

    def update(self):
        self.screen_.fill(self.WHITE_)

        # Draw hallway lines
        line_width = 50
        # Hallway connecting rooms 14 through 1 (bottom)
        pygame.draw.line(self.screen_, self.GRAY_, (180, 585), (1160, 585), line_width)
        # Hallway connecting rooms 14 through 22 (left side)
        pygame.draw.line(self.screen_, self.GRAY_, (240, 80), (240, 560), line_width)
        # Hallway connecting rooms 22 through 23 (top)
        pygame.draw.line(self.screen_, self.GRAY_, (215, 105), (1160, 105), line_width)
        # # Hallway for rooms 42 through 47
        # pygame.draw.line(self.screen_, self.GRAY_, (320, 525), (320, 225), line_width)
        # # Hallway for rooms 34 through 41
        # pygame.draw.line(self.screen_, self.GRAY_, (560, 525), (1120, 525), line_width)

        # Draw buttons and handle events
        mouse_pos = pygame.mouse.get_pos()
        for button in self.buttons_:
            pygame.draw.rect(self.screen_, button['color'], button['rect'])
            text_rect = button['text'].get_rect(center=button['rect'].center)
            self.screen_.blit(button['text'], text_rect)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                rclpy.shutdown()
            if event.type == pygame.MOUSEBUTTONDOWN:
                for button in self.buttons_:
                    if button['rect'].collidepoint(mouse_pos):
                        self.handle_navigation_request(button['label'], button['pose'])
                        break

        pygame.display.flip()

    def create_pose_stamped(self, position_x, position_y, rotation_z):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator_.get_clock().now().to_msg()
        goal_pose.pose.position.x = position_x
        goal_pose.pose.position.y = position_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = q_x
        goal_pose.pose.orientation.y = q_y
        goal_pose.pose.orientation.z = q_z
        goal_pose.pose.orientation.w = q_w
        return goal_pose

def main(args=None):
    rclpy.init(args=args)
    node = HmiNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
