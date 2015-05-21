# Contains basic stability information projected into the world frame
Header                 header

# Stability metric related to ratio of distance of CoP(ZMP) to boundary of support polygon to the sqrt of (foot area) 0=falling, 255=best single support
uint8                 stability

# Stance state
int8                  stance     # -1 left, 1 right, 2=double

# Fall detection
int8                  fall_state # -1 fallen, 0=stable upright, 1=currently falling

