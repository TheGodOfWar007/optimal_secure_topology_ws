# A special message type incorporating the pose of all the robots of
# the formation in an organized array with the UID alias field.
# This message is specifically for 2D pose

########################################
# Messages
########################################
pose2d_uid[] bot_sel

########################################
# CONSTANTS
########################################
# Utility data structure containing UIDs of the robots contained within bot_pose
# for avoiding querying the UID fields in bot_pose. Also the formation
# center UID.
string[] uid
string center_uid # The formation center bot's UID.
string common_frame_id

########################################
# FLAGS
########################################
uint8 USING_FORMATION_CENTER = 1
uint8 USING_COMMON_FRAME = 0
uint8 FORMATION_CENTER_IS_LEADER = 0
uint8 USING_LEADER_ELECTION = 0