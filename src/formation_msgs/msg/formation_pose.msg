# A special message type incorporating the pose of all the robots of
# the formation in an organized array with the UID alias field.
# This message is specifically for 2D pose

########################################
# Messages
########################################
pose_uid[] bot_sel

########################################
# CONSTANTS
########################################
# Utility data structure containing UIDs of the robots contained within bot_pose
# for avoiding querying the UID fields in bot_pose. Also the formation
# center UID.
string[] uid
string center_uid # The formation center.
string common_frame_id

########################################
# FLAGS
########################################
uint8 USING_FORMATION_CENTER # T/F
uint8 USING_COMMON_FRAME # T/F
uint8 FORMATION_CENTER_IS_LEADER # T/F
uint8 USING_LEADER_ELECTION # T/F