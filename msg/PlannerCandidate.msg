# ===================================================================
#  PlannerCandidate.msg
#  One candidate trajectory evaluated by the 30 Hz local planner.
#  Published (usually as an array of these) for the debug panel so the
#  rejected / selected candidates can be drawn.
# ===================================================================
std_msgs/Header header

uint32 index          # candidate index within the tick's sample set
bool   selected       # was this candidate chosen?
bool   rejected       # was this candidate rejected?
string reject_reason  # why it was rejected ("" if accepted)
float64 cost          # normalized cost (only meaningful for accepted)
float64 vx_body       # commanded body-forward velocity (m/s)
float64 vy_body       # commanded body-lateral velocity (m/s)
float64 yaw_rate      # commanded yaw rate (rad/s)
float64 min_clearance # min clearance to observed OCCUPIED cells (m)

float64[] traj_x      # world X of sampled trajectory points [N]
float64[] traj_y      # world Y of sampled trajectory points [N]
